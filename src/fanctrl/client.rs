//! fw-fanctrl socket client (design doc §2.1): a read-only `AF_UNIX` client
//! for `/run/fw-fanctrl/.fw-fanctrl.commands.sock`. One command per
//! connection: connect, send the raw CLI arg string, read to EOF, parse
//! JSON (`docs/research/05-fw-fanctrl-loop.md` §"The socket").
//!
//! **Read-only by construction.** [`PrintCommand`] has exactly two variants
//! (`Speed`, `All`) and there is no other way to send bytes down the socket
//! from this module — no raw-string send, no `set`/`use`/`pause` variant.
//! fw-fanctrl owns the fans; this client only ever asks it what it is doing.

use std::collections::HashMap;
use std::fmt;
use std::io::{Read, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

/// Default connect timeout (design doc §2.1).
pub const DEFAULT_CONNECT_TIMEOUT: Duration = Duration::from_secs(1);
/// Default read timeout (design doc §2.1). Applied *both* per read syscall
/// (`SO_RCVTIMEO`) and as a total deadline across the whole reply.
pub const DEFAULT_READ_TIMEOUT: Duration = Duration::from_secs(3);

/// Hard cap on one reply body. The real `print all` dump is ~8.8 KB
/// (`tests/fixtures/fanctrl/print_all_quiet16.json`); 1 MiB is ~100x that,
/// so no legitimate configuration can reach it, and a peer that never stops
/// writing cannot grow this process's heap.
const MAX_BODY: u64 = 1024 * 1024;

/// No successful `print speed` for this long ⇒ [`Freshness::Stale`] (design
/// doc §2.1), even while `print all` is within its own window.
const SPEED_STALE_AFTER: Duration = Duration::from_secs(15);
/// No successful `print all` for this long ⇒ [`Freshness::Stale`].
const ALL_STALE_AFTER: Duration = Duration::from_secs(90);

/// The only two commands this client can send. Closed by construction: a
/// later task must not be able to add a write/set/pause variant to this
/// enum even by mistake without touching this file, and every match on it
/// in this crate is exhaustive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrintCommand {
    Speed,
    All,
}

impl PrintCommand {
    /// The raw CLI arg string sent verbatim over the socket (research doc
    /// §"The socket": no framing, no auth, just this string).
    fn cli_string(self) -> &'static str {
        match self {
            PrintCommand::Speed => "--output-format JSON print speed",
            PrintCommand::All => "--output-format JSON print all",
        }
    }
}

/// A poll failure, classified for [`Freshness`] (design doc §2.1):
/// `Absent` (ENOENT / connection refused — fw-fanctrl is not reachable at
/// all) forces `Freshness::Absent` regardless of any prior successful poll;
/// `Timeout` and `Other` do not, since they mean the socket exists but this
/// one attempt failed — staleness is then judged purely from the view's
/// stamps (brief step 5: "all timing is computed from the view's monotonic
/// stamps").
#[derive(Debug)]
pub enum FanctrlError {
    /// Connect failed with ENOENT or ECONNREFUSED.
    Absent(String),
    /// Connect or read exceeded its timeout.
    Timeout,
    /// Any other I/O failure, or a response that connected fine but did not
    /// parse as the expected shape.
    Other(String),
}

impl fmt::Display for FanctrlError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FanctrlError::Absent(msg) => write!(f, "fw-fanctrl socket absent: {msg}"),
            FanctrlError::Timeout => write!(f, "fw-fanctrl socket poll timed out"),
            FanctrlError::Other(msg) => write!(f, "fw-fanctrl socket poll failed: {msg}"),
        }
    }
}

impl std::error::Error for FanctrlError {}

/// `Fresh` inside both staleness windows; `Stale` when either window has
/// elapsed (or has never once been satisfied — see [`FanctrlView`]);
/// `Absent` when the most recent poll attempt could not even connect
/// (ENOENT / connection refused), overriding the timing rule outright.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Freshness {
    Fresh,
    Stale,
    Absent,
}

/// fw-fanctrl's live state, as last observed over the socket (design doc
/// §2.1). Carries **two independent stamps**: `observed_at` is bumped by
/// *any* successful poll (`Speed` or `All`); `all_observed_at` only by a
/// successful `All`. A `Speed` poll only ever refreshes `speed_pct` and
/// `observed_at` — the rest of the fields (`strategy`, `curve`,
/// `temperature`, ...) are `All`-only data and keep their last known value
/// between `All` polls.
///
/// There is deliberately no "half a view" state: a view is only ever
/// created by a successful `All` (the only poll with enough data to build
/// one). A `Speed` success that arrives before the first `All` has nothing
/// to attach itself to and is dropped — see `UnixFanctrlClient::poll` and
/// `FakeFanctrl::poll`, which both document this the same way.
#[derive(Debug, Clone, PartialEq)]
pub struct FanctrlView {
    pub strategy: String,
    pub active: bool,
    pub speed_pct: u8,
    pub temperature: f64,
    pub ma_temperature: f64,
    /// `movingAverageInterval` of the currently resolved strategy (§Facts:
    /// 60 for both live curves on this machine).
    pub ma_interval: u32,
    /// Raw `(temperature, duty)` points of the resolved strategy, in file
    /// order — see [`resolve_curve`]. Consumers build a `Curve` from this
    /// with `Curve::from_points` (Task 1); this module does not depend on
    /// `curve.rs` at all, by design (§Global Constraints: don't widen
    /// scope by reaching into another task's API for no reason here).
    pub curve: Vec<(f64, u8)>,
    /// Stamped by any successful poll (`Speed` or `All`).
    pub observed_at: Instant,
    /// Stamped only by a successful `All`. `None` if `All` has never once
    /// succeeded (possible only in the window before the very first
    /// successful `All`, since a view cannot exist without one).
    pub all_observed_at: Option<Instant>,
}

/// The read-only abstraction the controller polls against — implemented by
/// [`UnixFanctrlClient`] (real socket I/O) and, in
/// `src/test_support/fakes.rs`, `FakeFanctrl` (scripted, for unit tests).
pub trait FanctrlSource {
    /// Sends `cmd`. On success, updates the internal view (see
    /// [`FanctrlView`]'s doc comment for exactly which fields each command
    /// touches) stamped with `now` and returns `Ok(())`. On failure, leaves
    /// the view untouched and returns the classified error; `now` is not
    /// used on the error path since no stamp is written.
    ///
    /// `now` is caller-supplied (rather than read internally via
    /// `Instant::now()`) so tests can drive freshness deterministically
    /// with synthetic future instants (`Instant::now() + Duration::from_secs(n)`)
    /// instead of real sleeps.
    fn poll(&mut self, cmd: PrintCommand, now: Instant) -> Result<(), FanctrlError>;

    /// Everything a *reader on another thread* needs, detached from `self`:
    /// the view plus the connect-failure flag that drives `Absent`. The
    /// poller thread publishes one of these into a small mutex after each
    /// round trip so the sampler tick never has to reach into a source that
    /// is (or may be) blocked inside socket I/O — see `sensors::poller`.
    fn snapshot(&self) -> FanctrlSnapshot;
}

/// A detached copy of a [`FanctrlSource`]'s observable state. Cheap to
/// publish (one assignment under a mutex that is never held across I/O) and
/// self-contained: [`Self::freshness`] applies the very same
/// [`compute_freshness`] rule the source itself would.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FanctrlSnapshot {
    pub view: Option<FanctrlView>,
    /// Mirrors `UnixFanctrlClient::last_absent`: the most recent poll
    /// attempt could not even connect (ENOENT/refused).
    pub last_absent: bool,
}

impl FanctrlSnapshot {
    /// [`Freshness`] of this snapshot as of `now` — identical rule to
    /// `FanctrlSource::freshness`, so a reader loses nothing by going
    /// through the published copy.
    pub fn freshness(&self, now: Instant) -> Freshness {
        compute_freshness(self.last_absent, self.view.as_ref(), now)
    }
}

/// Shared freshness rule (design doc §2.1), used by both
/// `UnixFanctrlClient` and `FakeFanctrl` (`test_support::fakes`) so the two
/// never drift apart. `pub(crate)` rather than a private `fn`: the fake
/// lives in a sibling module (`crate::test_support::fakes`), gated
/// `#[cfg(test)]` in `main.rs`, and needs this exact rule rather than a
/// second copy of it.
pub(crate) fn compute_freshness(
    last_absent: bool,
    view: Option<&FanctrlView>,
    now: Instant,
) -> Freshness {
    if last_absent {
        return Freshness::Absent;
    }
    let Some(view) = view else {
        // Never had a single successful poll, and the most recent attempt
        // (if any) was not ENOENT/refused — e.g. a read timeout on the very
        // first ever poll. Not "absent" (the socket is there), just stale.
        return Freshness::Stale;
    };
    let speed_fresh = now.saturating_duration_since(view.observed_at) < SPEED_STALE_AFTER;
    let all_fresh = view
        .all_observed_at
        .is_some_and(|t| now.saturating_duration_since(t) < ALL_STALE_AFTER);
    if speed_fresh && all_fresh {
        Freshness::Fresh
    } else {
        Freshness::Stale
    }
}

// --- JSON parsing -----------------------------------------------------

#[derive(serde::Deserialize)]
struct PrintAllResponse {
    strategy: String,
    active: bool,
    speed: u8,
    temperature: f64,
    #[serde(rename = "movingAverageTemperature")]
    moving_average_temperature: f64,
    configuration: ConfigurationField,
}

#[derive(serde::Deserialize)]
struct ConfigurationField {
    data: ConfigDataField,
}

#[derive(serde::Deserialize)]
struct ConfigDataField {
    /// **Deliberately untyped per entry.** fw-fanctrl's `print all` dumps the
    /// operator's *whole* configuration, and its own `config.schema.json`
    /// requires only `speedCurve` per strategy — so a schema-valid strategy
    /// the operator never activates may legitimately omit
    /// `movingAverageInterval`. Deserialising every entry strictly made one
    /// such inactive entry fail the entire poll, which stales
    /// `all_observed_at` after 90 s and drops the loop out of Mode A for good
    /// (roast PR-1 finding 9). Only the entry actually being looked up is
    /// parsed into a typed [`StrategyField`], and only that one has to be
    /// complete.
    strategies: HashMap<String, serde_json::Value>,
}

/// One strategy entry, parsed on demand out of the raw
/// `strategies[name]` JSON value — never for the map as a whole.
#[derive(serde::Deserialize)]
struct StrategyField {
    #[serde(rename = "movingAverageInterval")]
    moving_average_interval: u32,
}

/// The curve half of a strategy entry, kept separate from [`StrategyField`]
/// so `resolve_curve` can read a strategy's points even when that entry
/// omits `movingAverageInterval`.
#[derive(serde::Deserialize)]
struct StrategyCurveField {
    #[serde(rename = "speedCurve")]
    speed_curve: Vec<CurvePointField>,
}

#[derive(serde::Deserialize)]
struct CurvePointField {
    temp: f64,
    speed: u8,
}

#[derive(serde::Deserialize)]
struct PrintSpeedResponse {
    /// fw-fanctrl's `print speed` encodes the duty as a **string** (see
    /// `tests/fixtures/fanctrl/print_speed.json`: `"speed": "72"`), unlike
    /// `print all`'s numeric `speed` field — a real, observed asymmetry in
    /// the upstream protocol, not a typo here.
    speed: String,
}

/// `strategies[strategy].speedCurve` out of a raw `print all` JSON body, as
/// `(temperature, duty)` points in file order — design doc §2.1. Matches
/// the strategy name **exactly** (no fuzzy/case-insensitive matching): an
/// unknown name, or any JSON that doesn't parse as a `print all` response
/// at all, yields an empty list rather than an error, since a curve list is
/// already the "nothing resolved" signal callers need (`Curve::from_points`
/// on Task 1's side rejects an empty list on its own).
pub fn resolve_curve(print_all_json: &str, strategy: &str) -> Vec<(f64, u8)> {
    let Ok(parsed) = serde_json::from_str::<PrintAllResponse>(print_all_json) else {
        return Vec::new();
    };
    let Some(raw) = parsed.configuration.data.strategies.get(strategy) else {
        return Vec::new();
    };
    // Per-entry parse: an entry missing `speedCurve` (or shaped unexpectedly)
    // yields the same empty "nothing resolved" list a missing name does, and
    // never poisons the other entries.
    match serde_json::from_value::<StrategyCurveField>(raw.clone()) {
        Ok(s) => s.speed_curve.iter().map(|p| (p.temp, p.speed)).collect(),
        Err(_) => Vec::new(),
    }
}

/// Parses a raw `print all` JSON body into the `All`-sourced fields of a
/// [`FanctrlView`] (everything except the two stamps, which the caller
/// attaches). `ma_interval` comes from the **resolved** strategy's own
/// `movingAverageInterval` (nested under `configuration.data.strategies`),
/// not a top-level field.
fn parse_print_all(json: &str) -> Result<ParsedAll, FanctrlError> {
    let parsed: PrintAllResponse =
        serde_json::from_str(json).map_err(|e| FanctrlError::Other(e.to_string()))?;
    let Some(raw_entry) = parsed.configuration.data.strategies.get(&parsed.strategy) else {
        return Err(FanctrlError::Other(format!(
            "print all names active strategy {:?}, which is not in its own strategies map",
            parsed.strategy
        )));
    };
    // Only the *active* strategy has to be complete: the EC moving-average
    // emulator cannot run without its interval, so a missing one here is a
    // real error (degrading to the designed FANCTRL LOST / Mode B fallback)
    // rather than something to paper over with a guessed default.
    let entry: StrategyField = serde_json::from_value(raw_entry.clone()).map_err(|e| {
        FanctrlError::Other(format!(
            "print all's active strategy {:?} did not parse: {e}",
            parsed.strategy
        ))
    })?;
    let ma_interval = entry.moving_average_interval;
    // Single source of truth for "strategy name -> curve points" (design
    // §2.1): this production path and every direct `resolve_curve` caller
    // share the same lookup instead of a second copy of the
    // `speedCurve` mapping drifting from it.
    let curve = resolve_curve(json, &parsed.strategy);
    Ok(ParsedAll {
        strategy: parsed.strategy,
        active: parsed.active,
        speed_pct: parsed.speed,
        temperature: parsed.temperature,
        ma_temperature: parsed.moving_average_temperature,
        ma_interval,
        curve,
    })
}

/// The `All`-sourced fields of a [`FanctrlView`], before the caller attaches
/// its own stamps.
#[derive(Debug)]
struct ParsedAll {
    strategy: String,
    active: bool,
    speed_pct: u8,
    temperature: f64,
    ma_temperature: f64,
    ma_interval: u32,
    curve: Vec<(f64, u8)>,
}

/// Parses a raw `print speed` JSON body into the commanded duty.
fn parse_print_speed(json: &str) -> Result<u8, FanctrlError> {
    let parsed: PrintSpeedResponse =
        serde_json::from_str(json).map_err(|e| FanctrlError::Other(e.to_string()))?;
    parsed.speed.parse::<u8>().map_err(|e| {
        FanctrlError::Other(format!("print speed {:?} did not parse: {e}", parsed.speed))
    })
}

// --- UnixFanctrlClient --------------------------------------------------

/// Classifies a raw I/O error into a [`FanctrlError`] (design doc §2.1:
/// "Connection refused / ENOENT ⇒ absent").
fn classify_io_error(e: std::io::Error) -> FanctrlError {
    match e.kind() {
        std::io::ErrorKind::NotFound | std::io::ErrorKind::ConnectionRefused => {
            FanctrlError::Absent(e.to_string())
        }
        std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock => FanctrlError::Timeout,
        _ => FanctrlError::Other(e.to_string()),
    }
}

/// Connects to `path`, bounded by `timeout`. `UnixStream::connect` has no
/// built-in timeout (unlike `TcpStream::connect_timeout`); in practice an
/// `AF_UNIX` connect is immediate (instant success, or an immediate
/// ENOENT/ECONNREFUSED when nothing is listening) so this thread-plus-
/// channel wrapper only guards the pathological case (a full accept
/// backlog) — it never adds latency to the common path, and it never adds a
/// new crate dependency (`std::sync::mpsc` + `std::thread` only). If the
/// spawned connect never returns, the thread outlives this call (there is
/// no way to cancel a blocked syscall from the outside in std alone); its
/// eventual `tx.send` then silently fails since `rx` has already been
/// dropped, and the thread exits.
///
/// Uses `thread::Builder::spawn` rather than `thread::spawn`: the latter
/// *panics* if the OS refuses to create the thread, and this runs on the
/// poller thread with the shared snapshot lock available to the sampler — a
/// panic here used to poison it (roast PR-1 finding 3). OS refusal is
/// reported as a plain `Other` error, i.e. one failed poll.
fn connect_with_timeout(path: &Path, timeout: Duration) -> Result<UnixStream, FanctrlError> {
    let (tx, rx) = mpsc::channel();
    let owned = path.to_path_buf();
    thread::Builder::new()
        .name("fanctrl-connect".into())
        .spawn(move || {
            let _ = tx.send(UnixStream::connect(&owned));
        })
        .map_err(|e| FanctrlError::Other(format!("could not spawn fanctrl connect thread: {e}")))?;
    match rx.recv_timeout(timeout) {
        Ok(Ok(stream)) => Ok(stream),
        Ok(Err(e)) => Err(classify_io_error(e)),
        Err(_) => Err(FanctrlError::Timeout),
    }
}

/// The real socket client: one `AF_UNIX` connection per command, to
/// `fanctrl_socket` (config key, default
/// `/run/fw-fanctrl/.fw-fanctrl.commands.sock`).
pub struct UnixFanctrlClient {
    socket_path: PathBuf,
    connect_timeout: Duration,
    read_timeout: Duration,
    view: Option<FanctrlView>,
    /// Set when the most recent poll attempt failed to connect at all
    /// (ENOENT/refused); cleared by any poll that at least connects,
    /// success or not. Drives `Freshness::Absent` independently of the
    /// view's stamps — see `compute_freshness`.
    last_absent: bool,
}

impl UnixFanctrlClient {
    /// Client with the default 1 s connect / 3 s read timeouts (design doc
    /// §2.1).
    pub fn new(socket_path: PathBuf) -> Self {
        UnixFanctrlClient {
            socket_path,
            connect_timeout: DEFAULT_CONNECT_TIMEOUT,
            read_timeout: DEFAULT_READ_TIMEOUT,
            view: None,
            last_absent: false,
        }
    }

    /// Same as [`Self::new`] with explicit timeouts — used by this module's
    /// own tests to keep the read-timeout case fast rather than waiting out
    /// the real 3 s default.
    #[cfg(test)]
    fn with_timeouts(
        socket_path: PathBuf,
        connect_timeout: Duration,
        read_timeout: Duration,
    ) -> Self {
        UnixFanctrlClient {
            socket_path,
            connect_timeout,
            read_timeout,
            view: None,
            last_absent: false,
        }
    }

    /// The last-known view, or `None` if no poll has ever succeeded.
    /// Inherent rather than part of [`FanctrlSource`]: cross-thread readers
    /// go through the published [`FanctrlSnapshot`] instead (see
    /// `sensors::poller`), so the trait exposes only `poll`/`snapshot`.
    #[cfg(test)]
    fn view(&self) -> Option<&FanctrlView> {
        self.view.as_ref()
    }

    /// [`Freshness`] of `view()` as of `now`. Never panics regardless of how
    /// `now` relates to the view's stamps (uses saturating duration
    /// arithmetic), so a caller may safely pass a clock that has jumped.
    #[cfg(test)]
    fn freshness(&self, now: Instant) -> Freshness {
        compute_freshness(self.last_absent, self.view.as_ref(), now)
    }

    /// One full round trip: connect, send `cmd`'s raw CLI string, shut down
    /// the write half (so a server reading to EOF sees the command end),
    /// read the response to EOF — bounded by both [`MAX_BODY`] and a *total*
    /// read deadline of `read_timeout`.
    ///
    /// The two bounds are not redundant with `set_read_timeout`: `SO_RCVTIMEO`
    /// applies to each individual read syscall, so a peer trickling one byte
    /// per interval kept the old `read_to_string`-to-EOF loop running
    /// unboundedly, and a peer that never stops writing grew `body`
    /// unboundedly (roast PR-1 finding 2).
    fn send(&self, cmd: PrintCommand) -> Result<String, FanctrlError> {
        let mut stream = connect_with_timeout(&self.socket_path, self.connect_timeout)?;
        stream
            .set_read_timeout(Some(self.read_timeout))
            .map_err(classify_io_error)?;
        stream
            .set_write_timeout(Some(self.read_timeout))
            .map_err(classify_io_error)?;
        stream
            .write_all(cmd.cli_string().as_bytes())
            .map_err(classify_io_error)?;
        // Signal EOF on our side so a server that reads-to-EOF-then-
        // responds is not left waiting for more input that never comes.
        let _ = stream.shutdown(std::net::Shutdown::Write);
        self.read_body(stream)
    }

    /// The bounded read half of [`Self::send`]: reads to EOF, but never past
    /// [`MAX_BODY`] bytes and never past a total deadline of `read_timeout`
    /// from the moment the read phase begins.
    fn read_body(&self, stream: UnixStream) -> Result<String, FanctrlError> {
        let deadline = Instant::now() + self.read_timeout;
        // +1 so an exactly-MAX_BODY body still reads its real EOF, while the
        // first byte past the cap is observable rather than silently
        // truncated into a body that would then "just" fail to parse.
        let mut limited = stream.take(MAX_BODY + 1);
        let mut body: Vec<u8> = Vec::new();
        let mut chunk = [0u8; 8192];
        loop {
            if Instant::now() >= deadline {
                return Err(FanctrlError::Timeout);
            }
            match limited.read(&mut chunk) {
                Ok(0) => break,
                Ok(n) => {
                    body.extend_from_slice(&chunk[..n]);
                    if body.len() as u64 > MAX_BODY {
                        return Err(FanctrlError::Other(format!(
                            "reply exceeded the {MAX_BODY} byte cap"
                        )));
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::Interrupted => {}
                Err(e) => return Err(classify_io_error(e)),
            }
        }
        String::from_utf8(body)
            .map_err(|e| FanctrlError::Other(format!("reply was not valid UTF-8: {e}")))
    }
}

impl FanctrlSource for UnixFanctrlClient {
    fn poll(&mut self, cmd: PrintCommand, now: Instant) -> Result<(), FanctrlError> {
        match self.send(cmd) {
            Ok(body) => {
                self.last_absent = false;
                match cmd {
                    PrintCommand::All => {
                        let parsed = parse_print_all(&body)?;
                        self.view = Some(FanctrlView {
                            strategy: parsed.strategy,
                            active: parsed.active,
                            speed_pct: parsed.speed_pct,
                            temperature: parsed.temperature,
                            ma_temperature: parsed.ma_temperature,
                            ma_interval: parsed.ma_interval,
                            curve: parsed.curve,
                            observed_at: now,
                            all_observed_at: Some(now),
                        });
                    }
                    PrintCommand::Speed => {
                        let speed_pct = parse_print_speed(&body)?;
                        // See FanctrlView's doc comment: a Speed-only
                        // success before the first All has nothing to
                        // attach to and is dropped.
                        if let Some(view) = &mut self.view {
                            view.speed_pct = speed_pct;
                            view.observed_at = now;
                        }
                    }
                }
                Ok(())
            }
            Err(e) => {
                self.last_absent = matches!(e, FanctrlError::Absent(_));
                Err(e)
            }
        }
    }

    fn snapshot(&self) -> FanctrlSnapshot {
        FanctrlSnapshot {
            view: self.view.clone(),
            last_absent: self.last_absent,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::net::UnixListener;

    // --- Step 2: parsing print all --------------------------------------

    fn fixture(rel: &str) -> String {
        std::fs::read_to_string(crate::test_support::fixtures::path(rel)).unwrap()
    }

    #[test]
    fn parses_print_all_quiet16_fixture() {
        let json = fixture("fanctrl/print_all_quiet16.json");
        let parsed = parse_print_all(&json).unwrap();
        assert_eq!(parsed.strategy, "quiet16");
        assert!(parsed.active);
        assert_eq!(parsed.speed_pct, 32);
        assert_eq!(parsed.temperature, 77.0);
        assert_eq!(parsed.ma_interval, 60);
    }

    #[test]
    fn parses_print_all_cool16_fixture() {
        let json = fixture("fanctrl/print_all_cool16.json");
        let parsed = parse_print_all(&json).unwrap();
        assert_eq!(parsed.strategy, "cool16");
        assert!(parsed.active);
        assert_eq!(parsed.speed_pct, 72);
        assert_eq!(parsed.temperature, 78.0);
        assert_eq!(parsed.ma_interval, 60);
    }

    #[test]
    fn parse_print_all_rejects_garbage() {
        let err = parse_print_all("not json").unwrap_err();
        assert!(matches!(err, FanctrlError::Other(_)));
    }

    // --- Step 3: resolve_curve -------------------------------------------

    #[test]
    fn resolve_curve_yields_exact_quiet16_points() {
        let json = fixture("fanctrl/print_all_quiet16.json");
        let points = resolve_curve(&json, "quiet16");
        assert_eq!(
            points,
            vec![
                (0.0, 15),
                (55.0, 15),
                (65.0, 21),
                (75.0, 31),
                (82.0, 37),
                (88.0, 55),
                (95.0, 100),
            ]
        );
    }

    #[test]
    fn resolve_curve_yields_exact_cool16_points() {
        // cool16 is present (as a non-active strategy) in the quiet16
        // fixture's own `strategies` map too, since fw-fanctrl always
        // reports the full config regardless of which strategy is live.
        let json = fixture("fanctrl/print_all_quiet16.json");
        let points = resolve_curve(&json, "cool16");
        assert_eq!(
            points,
            vec![(0.0, 20), (50.0, 20), (60.0, 30), (70.0, 42), (85.0, 100)]
        );
    }

    #[test]
    fn resolve_curve_unknown_strategy_yields_empty() {
        let json = fixture("fanctrl/print_all_quiet16.json");
        assert_eq!(resolve_curve(&json, "QUIET16"), Vec::new()); // case must not match
        assert_eq!(resolve_curve(&json, "nonexistent"), Vec::new());
    }

    #[test]
    fn resolve_curve_on_unparseable_json_yields_empty() {
        assert_eq!(resolve_curve("not json", "quiet16"), Vec::new());
    }

    // --- Step 6: parse_print_speed (string-encoded duty) -----------------

    #[test]
    fn parses_print_speed_fixture() {
        let json = fixture("fanctrl/print_speed.json");
        assert_eq!(parse_print_speed(&json).unwrap(), 72);
    }

    #[test]
    fn parse_print_speed_rejects_non_numeric_string() {
        let err = parse_print_speed(r#"{"status":"success","speed":"nope"}"#).unwrap_err();
        assert!(matches!(err, FanctrlError::Other(_)));
    }

    // --- Step 4: the two stamps -------------------------------------------

    fn all_view(now: Instant) -> FanctrlView {
        FanctrlView {
            strategy: "quiet16".to_string(),
            active: true,
            speed_pct: 31,
            temperature: 75.0,
            ma_temperature: 75.0,
            ma_interval: 60,
            curve: vec![(0.0, 15), (95.0, 100)],
            observed_at: now,
            all_observed_at: Some(now),
        }
    }

    #[test]
    fn a_speed_refresh_bumps_observed_at_but_not_all_observed_at() {
        let t0 = Instant::now();
        let mut view = all_view(t0);
        let t1 = t0 + Duration::from_secs(5);
        // Emulate exactly what UnixFanctrlClient::poll does on a Speed
        // success against an existing view.
        view.speed_pct = 40;
        view.observed_at = t1;
        assert_eq!(view.observed_at, t1);
        assert_eq!(view.all_observed_at, Some(t0));
        assert_eq!(view.speed_pct, 40);
    }

    #[test]
    fn an_all_refresh_bumps_both_stamps() {
        let t0 = Instant::now();
        let t1 = t0 + Duration::from_secs(30);
        let view = all_view(t1);
        assert_eq!(view.observed_at, t1);
        assert_eq!(view.all_observed_at, Some(t1));
    }

    // --- Step 5: Freshness -------------------------------------------------

    #[test]
    fn freshness_is_fresh_inside_both_windows() {
        let t0 = Instant::now();
        let view = all_view(t0);
        let now = t0 + Duration::from_secs(10); // < 15s, < 90s
        assert_eq!(compute_freshness(false, Some(&view), now), Freshness::Fresh);
    }

    #[test]
    fn freshness_is_stale_when_print_all_exceeds_90s() {
        let t0 = Instant::now();
        let view = all_view(t0);
        let now = t0 + Duration::from_secs(91);
        assert_eq!(compute_freshness(false, Some(&view), now), Freshness::Stale);
    }

    #[test]
    fn freshness_is_stale_when_print_speed_fails_for_15s_even_if_all_is_fresh() {
        let t0 = Instant::now();
        let mut view = all_view(t0);
        // print all stays fresh (all_observed_at untouched at t0); print
        // speed has not refreshed observed_at since t0 either, and 16s have
        // now passed — the 15s speed window is what should trip Stale even
        // though the 90s all window is nowhere close.
        view.all_observed_at = Some(t0);
        let now = t0 + Duration::from_secs(16);
        assert_eq!(compute_freshness(false, Some(&view), now), Freshness::Stale);
    }

    #[test]
    fn freshness_is_absent_when_last_poll_could_not_connect() {
        let t0 = Instant::now();
        let view = all_view(t0);
        // Even a perfectly fresh view is overridden: the daemon is gone
        // *right now*, which is a stronger statement than "the data is
        // getting old".
        assert_eq!(compute_freshness(true, Some(&view), t0), Freshness::Absent);
    }

    #[test]
    fn freshness_is_stale_not_absent_with_no_view_and_no_connect_failure() {
        // The "read timeout on the very first ever poll" case: nothing has
        // ever been observed, but the most recent failure was not
        // ENOENT/refused (the socket exists, it's just slow).
        assert_eq!(
            compute_freshness(false, None, Instant::now()),
            Freshness::Stale
        );
    }

    // --- Step 6/7: real socket I/O (UnixFanctrlClient) ---------------------

    /// Unique-per-test socket path; short (AF_UNIX paths are capped around
    /// 108 bytes on Linux), inside the system temp dir.
    fn socket_path(name: &str) -> PathBuf {
        static COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        std::env::temp_dir().join(format!("bzf-{}-{name}-{n}.sock", std::process::id()))
    }

    /// Spawns a one-shot `AF_UNIX` server: accepts a single connection,
    /// reads whatever the client sends (to EOF), then runs `respond` on the
    /// resulting stream (write a canned reply, or sleep to force a client
    /// read timeout). Returns the bound path, already listening by the time
    /// this returns — `UnixListener::bind` runs synchronously on the
    /// *caller's* thread, only `accept` moves to the background one, so a
    /// `connect()` right after this call races nothing.
    fn spawn_one_shot_server(respond: impl FnOnce(UnixStream) + Send + 'static) -> PathBuf {
        let path = socket_path("srv");
        let listener = UnixListener::bind(&path).unwrap();
        thread::spawn(move || {
            if let Ok((stream, _)) = listener.accept() {
                respond(stream);
            }
        });
        path
    }

    /// Same idea as [`spawn_one_shot_server`], but accepts `responders.len()`
    /// connections in order (one per `PrintCommand::poll` call a test makes
    /// against the same client, which always reconnects to the same
    /// `socket_path`) instead of exactly one.
    fn spawn_sequenced_server(responders: Vec<Box<dyn FnOnce(UnixStream) + Send>>) -> PathBuf {
        let path = socket_path("seq");
        let listener = UnixListener::bind(&path).unwrap();
        thread::spawn(move || {
            for respond in responders {
                if let Ok((stream, _)) = listener.accept() {
                    respond(stream);
                } else {
                    break;
                }
            }
        });
        path
    }

    #[test]
    fn unix_client_parses_a_real_print_all_round_trip() {
        let body = fixture("fanctrl/print_all_quiet16.json");
        let path = spawn_one_shot_server(move |mut stream| {
            let mut sent = String::new();
            let _ = stream.read_to_string(&mut sent);
            assert_eq!(sent, PrintCommand::All.cli_string());
            let _ = stream.write_all(body.as_bytes());
        });
        let mut client = UnixFanctrlClient::new(path.clone());
        let now = Instant::now();
        client.poll(PrintCommand::All, now).unwrap();
        let view = client.view().unwrap();
        assert_eq!(view.strategy, "quiet16");
        assert_eq!(view.speed_pct, 32);
        assert_eq!(view.observed_at, now);
        assert_eq!(view.all_observed_at, Some(now));
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn unix_client_speed_refresh_bumps_observed_at_but_not_all_observed_at() {
        // Exercises UnixFanctrlClient::poll's own Speed-arm end to end over a
        // real socket — not just FakeFanctrl's separate implementation of
        // the same rule (test_support::fakes has its own coverage of this
        // for the fake).
        let all_body = fixture("fanctrl/print_all_quiet16.json");
        let speed_body = fixture("fanctrl/print_speed.json"); // speed: "72"
        let path = spawn_sequenced_server(vec![
            Box::new(move |mut stream| {
                let mut sent = String::new();
                let _ = stream.read_to_string(&mut sent);
                assert_eq!(sent, PrintCommand::All.cli_string());
                let _ = stream.write_all(all_body.as_bytes());
            }),
            Box::new(move |mut stream| {
                let mut sent = String::new();
                let _ = stream.read_to_string(&mut sent);
                assert_eq!(sent, PrintCommand::Speed.cli_string());
                let _ = stream.write_all(speed_body.as_bytes());
            }),
        ]);
        let mut client = UnixFanctrlClient::new(path.clone());
        let t0 = Instant::now();
        client.poll(PrintCommand::All, t0).unwrap();
        let t1 = t0 + Duration::from_secs(5);
        client.poll(PrintCommand::Speed, t1).unwrap();

        let view = client.view().unwrap();
        assert_eq!(view.observed_at, t1, "Speed poll must bump observed_at");
        assert_eq!(
            view.all_observed_at,
            Some(t0),
            "Speed poll must NOT bump all_observed_at"
        );
        assert_eq!(view.speed_pct, 72, "Speed poll must update speed_pct");
        // All-only fields keep their value from the earlier All poll.
        assert_eq!(view.strategy, "quiet16");
        assert_eq!(view.temperature, 77.0);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn unix_client_enoent_yields_absent_error_and_freshness() {
        // No listener was ever bound at this path.
        let path = socket_path("enoent");
        let mut client = UnixFanctrlClient::with_timeouts(
            path,
            Duration::from_millis(200),
            Duration::from_millis(200),
        );
        let err = client.poll(PrintCommand::All, Instant::now()).unwrap_err();
        assert!(matches!(err, FanctrlError::Absent(_)), "got {err:?}");
        assert_eq!(client.freshness(Instant::now()), Freshness::Absent);
    }

    #[test]
    fn unix_client_read_timeout_yields_timeout_error_and_stale_freshness() {
        // The server accepts and reads the command, but never writes a
        // reply: the client's read must time out rather than hang.
        let path = spawn_one_shot_server(|mut stream| {
            let mut sent = String::new();
            let _ = stream.read_to_string(&mut sent);
            thread::sleep(Duration::from_secs(2)); // outlives the client's read timeout
        });
        let mut client = UnixFanctrlClient::with_timeouts(
            path.clone(),
            Duration::from_millis(200),
            Duration::from_millis(200),
        );
        let err = client.poll(PrintCommand::All, Instant::now()).unwrap_err();
        assert!(matches!(err, FanctrlError::Timeout), "got {err:?}");
        assert_eq!(client.freshness(Instant::now()), Freshness::Stale);
        let _ = std::fs::remove_file(&path);
    }

    // --- roast PR-1 finding 2: bounded reads --------------------------------

    #[test]
    fn a_trickling_peer_hits_the_total_read_deadline_instead_of_reading_forever() {
        // `SO_RCVTIMEO` bounds each individual read, not the whole reply, so
        // a peer that dribbles one byte per interval below that timeout kept
        // the old `read_to_string`-to-EOF loop running for as long as it
        // cared to keep dribbling. The total deadline must cut it off.
        let path = spawn_one_shot_server(|mut stream| {
            let mut sent = String::new();
            let _ = stream.read_to_string(&mut sent);
            // 100 x 50 ms = ~5 s of trickle, every gap comfortably inside the
            // 300 ms per-read timeout below.
            for _ in 0..100 {
                if stream.write_all(b" ").is_err() {
                    return;
                }
                thread::sleep(Duration::from_millis(50));
            }
        });
        let mut client = UnixFanctrlClient::with_timeouts(
            path.clone(),
            Duration::from_millis(200),
            Duration::from_millis(300),
        );
        let started = Instant::now();
        let err = client.poll(PrintCommand::All, Instant::now()).unwrap_err();
        let elapsed = started.elapsed();
        assert!(matches!(err, FanctrlError::Timeout), "got {err:?}");
        assert!(
            elapsed < Duration::from_secs(2),
            "the trickle ran for {elapsed:?}: the total read deadline was not enforced"
        );
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn an_oversized_reply_is_refused_at_the_body_cap() {
        // A peer that never stops writing must not be able to grow this
        // process's heap: the reply is capped, and the cap is reported as
        // its own error rather than as a truncated body that merely fails
        // to parse.
        let path = spawn_one_shot_server(|mut stream| {
            let mut sent = String::new();
            let _ = stream.read_to_string(&mut sent);
            let chunk = vec![b'x'; 64 * 1024];
            // 4 MiB, i.e. four times the cap.
            for _ in 0..64 {
                if stream.write_all(&chunk).is_err() {
                    return;
                }
            }
        });
        let mut client = UnixFanctrlClient::with_timeouts(
            path.clone(),
            Duration::from_millis(200),
            Duration::from_secs(5),
        );
        let err = client.poll(PrintCommand::All, Instant::now()).unwrap_err();
        match err {
            FanctrlError::Other(msg) => assert!(
                msg.contains("exceeded") && msg.contains("cap"),
                "expected the body-cap error, got {msg:?}"
            ),
            other => panic!("expected the body-cap error, got {other:?}"),
        }
        let _ = std::fs::remove_file(&path);
    }

    // --- roast PR-1 finding 9: lenient inactive strategies -------------------

    /// A `print all` body whose *inactive* `userCustom` strategy omits
    /// `movingAverageInterval` -- schema-valid upstream (fw-fanctrl's
    /// `config.schema.json` requires only `speedCurve`) and produced verbatim
    /// by `dump_details`, since it returns the operator's raw configuration.
    const PRINT_ALL_INACTIVE_STRATEGY_MISSING_MA: &str = r#"{
      "strategy": "quiet16",
      "active": true,
      "speed": 32,
      "temperature": 77.0,
      "movingAverageTemperature": 76.5,
      "configuration": { "data": { "strategies": {
        "quiet16": {
          "movingAverageInterval": 60,
          "speedCurve": [{"temp": 0, "speed": 15}, {"temp": 95, "speed": 100}]
        },
        "userCustom": {
          "speedCurve": [{"temp": 0, "speed": 20}, {"temp": 80, "speed": 90}]
        }
      } } }
    }"#;

    #[test]
    fn an_inactive_strategy_missing_moving_average_interval_does_not_fail_the_poll() {
        // Before the fix every entry in the map was deserialised strictly, so
        // this one unrelated entry failed the whole `print all` -- staling
        // `all_observed_at` after 90 s and dropping the loop out of Mode A
        // permanently, even though only the active strategy's data is used.
        let parsed = parse_print_all(PRINT_ALL_INACTIVE_STRATEGY_MISSING_MA)
            .expect("an incomplete inactive strategy must not fail the poll");
        assert_eq!(parsed.strategy, "quiet16");
        assert_eq!(parsed.ma_interval, 60);
        assert_eq!(parsed.curve, vec![(0.0, 15), (95.0, 100)]);
    }

    #[test]
    fn an_incomplete_inactive_strategys_curve_still_resolves() {
        assert_eq!(
            resolve_curve(PRINT_ALL_INACTIVE_STRATEGY_MISSING_MA, "userCustom"),
            vec![(0.0, 20), (80.0, 90)]
        );
    }

    #[test]
    fn the_active_strategy_missing_moving_average_interval_is_still_an_error() {
        // The EC moving-average emulator cannot run without the interval, so
        // this degrades to the designed FANCTRL LOST / Mode B fallback rather
        // than being papered over with a guessed default.
        let json =
            PRINT_ALL_INACTIVE_STRATEGY_MISSING_MA.replace("\"movingAverageInterval\": 60,", "");
        let err = parse_print_all(&json).unwrap_err();
        assert!(matches!(err, FanctrlError::Other(_)), "got {err:?}");
    }

    // --- Step 7: PrintCommand is exactly two variants -----------------------

    #[test]
    fn print_command_has_exactly_two_variants_with_the_documented_cli_strings() {
        // Exhaustive match: if a third variant is ever added, this match
        // stops compiling, which is the whole point (Global Constraints:
        // "the socket client may send only print speed and print all").
        for cmd in [PrintCommand::Speed, PrintCommand::All] {
            match cmd {
                PrintCommand::Speed => {
                    assert_eq!(cmd.cli_string(), "--output-format JSON print speed");
                }
                PrintCommand::All => {
                    assert_eq!(cmd.cli_string(), "--output-format JSON print all");
                }
            }
        }
    }
}
