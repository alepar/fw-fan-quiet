//! File-only tracing setup: stdout belongs to the TUI, so logs go to a daily
//! rolling file under the log dir (falling back to the current directory).

use std::path::Path;

use tracing_appender::non_blocking::WorkerGuard;
use tracing_appender::rolling::{InitError, RollingFileAppender, Rotation};
use tracing_subscriber::EnvFilter;

/// Initializes the global tracing subscriber writing to
/// `<log_dir>/bazerame-fans.<date>.log`. Filter defaults to `info`,
/// overridable via `RUST_LOG`. Returns the appender's worker guard — keep it
/// alive in main or buffered log lines are lost. `None` if no log file could
/// be opened anywhere or a subscriber is already installed.
pub fn init(log_dir: &Path) -> Option<WorkerGuard> {
    let appender = build_appender(log_dir)
        .or_else(|_| build_appender(Path::new(".")))
        .ok()?;
    let (writer, guard) = tracing_appender::non_blocking(appender);
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_writer(writer)
        .with_ansi(false) // it's a file, not a terminal
        .try_init()
        .ok()?;
    Some(guard)
}

fn build_appender(dir: &Path) -> Result<RollingFileAppender, InitError> {
    // The builder does not create missing directories; best-effort here and
    // let build() report the real error if the dir is still unusable.
    let _ = std::fs::create_dir_all(dir);
    RollingFileAppender::builder()
        .rotation(Rotation::DAILY)
        .filename_prefix("bazerame-fans")
        .filename_suffix("log")
        .build(dir)
}
