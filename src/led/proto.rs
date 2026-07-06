//! Framework LED Matrix serial protocol over USB CDC-ACM. Commands are framed
//! as `0x32 0xAC <cmd> <args...>` (max 64 bytes); a full 9x34 grayscale grid
//! is pushed by staging each column (`SendCol`) then applying them all at once
//! (`CommitCols`). Reference: FrameworkComputer/inputmodule-rs.

use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::path::Path;

use crate::led::render::{HEIGHT, WIDTH};

/// Every command frame starts with these two magic bytes.
const MAGIC: [u8; 2] = [0x32, 0xAC];
/// Set the module's global PWM brightness (arg: one byte, 0-255).
const CMD_BRIGHTNESS: u8 = 0x00;
/// Enter/leave the low-power sleep state (arg: 1 = sleep, 0 = wake).
const CMD_SLEEP: u8 = 0x03;
/// Stage one column's grayscale values (arg: column index + 34 bytes).
const CMD_SEND_COL: u8 = 0x07;
/// Apply all staged columns to the display at once (no args).
const CMD_COMMIT: u8 = 0x08;

/// Builds one protocol frame into `buf`, returning the used slice. Split out
/// from the I/O so the byte layout is unit-testable without a device.
/// `args.len()` must be <= 61 (64 - 2 magic - 1 command); the LED commands we
/// send top out at 35 (`SendCol`: 1 index + 34 rows).
fn frame<'a>(buf: &'a mut [u8; 64], cmd: u8, args: &[u8]) -> &'a [u8] {
    buf[0..2].copy_from_slice(&MAGIC);
    buf[2] = cmd;
    let end = 3 + args.len();
    buf[3..end].copy_from_slice(args);
    &buf[..end]
}

/// An open LED matrix module (one serial port). Every write is fallible and
/// bubbles up so the caller can drop the module on error; nothing here can
/// touch the fan-control path. Orientation lives in the (pure) renderer, so
/// this type is a literal grid transport.
pub struct Matrix {
    port: File,
}

impl Matrix {
    /// Opens `path` (a `/dev/ttyACM*` node or `by-path` symlink) in raw mode
    /// and sets the global brightness. Raw mode is mandatory: the default tty
    /// line discipline would translate bytes like `0x0A` in our binary frames.
    pub fn open(path: &Path, brightness: u8) -> io::Result<Self> {
        let port = OpenOptions::new()
            .read(true)
            .write(true)
            // O_NOCTTY: never adopt the LED module as our controlling terminal.
            .custom_flags(libc::O_NOCTTY)
            .open(path)?;
        make_raw(port.as_raw_fd())?;
        let mut matrix = Matrix { port };
        // Wake first: a module idle past its inactivity timeout ignores draw
        // commands until woken, so the very first frame would otherwise be
        // dropped. The 1 Hz redraw keeps it awake thereafter.
        matrix.command(CMD_SLEEP, &[0])?;
        matrix.command(CMD_BRIGHTNESS, &[brightness])?;
        Ok(matrix)
    }

    fn command(&mut self, cmd: u8, args: &[u8]) -> io::Result<()> {
        let mut buf = [0u8; 64];
        let frame = frame(&mut buf, cmd, args);
        self.port.write_all(frame)
    }

    /// Draws a full `[column][row]` grid: stages each of the 9 columns then
    /// commits them together. `grid[x]` is column x's 34 grayscale values.
    pub fn draw_grid(&mut self, grid: &[[u8; HEIGHT]; WIDTH]) -> io::Result<()> {
        let mut arg = [0u8; 1 + HEIGHT];
        for (x, column) in grid.iter().enumerate() {
            arg[0] = x as u8;
            arg[1..].copy_from_slice(column);
            self.command(CMD_SEND_COL, &arg)?;
        }
        self.command(CMD_COMMIT, &[])
    }

    /// Turns every LED off (used on clean shutdown so the panel does not freeze
    /// on the last frame).
    pub fn blank(&mut self) -> io::Result<()> {
        self.draw_grid(&[[0u8; HEIGHT]; WIDTH])
    }
}

/// Puts a tty into raw mode: no input/output/line processing, 8N1, and
/// `CLOCAL | CREAD` so a USB CDC-ACM open never blocks waiting on carrier.
fn make_raw(fd: std::os::fd::RawFd) -> io::Result<()> {
    use termios::{CLOCAL, CREAD, TCSANOW, Termios, cfmakeraw, tcsetattr};
    let mut t = Termios::from_fd(fd)?;
    cfmakeraw(&mut t);
    t.c_cflag |= CLOCAL | CREAD;
    tcsetattr(fd, TCSANOW, &t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_layout_has_magic_command_and_args() {
        let mut buf = [0u8; 64];
        let out = frame(&mut buf, CMD_SEND_COL, &[2, 10, 20, 30]);
        assert_eq!(out, &[0x32, 0xAC, 0x07, 2, 10, 20, 30]);
    }

    #[test]
    fn frame_with_no_args_is_just_the_header() {
        let mut buf = [0u8; 64];
        let out = frame(&mut buf, CMD_COMMIT, &[]);
        assert_eq!(out, &[0x32, 0xAC, 0x08]);
    }

    #[test]
    fn send_col_frame_fits_the_64_byte_budget() {
        // Worst case: column index + 34 rows = 35 args, +3 header = 38 bytes.
        let mut buf = [0u8; 64];
        let out = frame(&mut buf, CMD_SEND_COL, &[0u8; 1 + HEIGHT]);
        assert_eq!(out.len(), 3 + 1 + HEIGHT);
        assert!(out.len() <= 64);
    }
}
