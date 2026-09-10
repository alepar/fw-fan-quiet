# Task 7 report: fw-fanctrl socket client (fw-fanctrl-loop-58u)

## What I implemented

A read-only `AF_UNIX` socket client for `/run/fw-fanctrl/.fw-fanctrl.commands.sock`
(design doc §2.1), plus a scriptable fake for downstream tests, in
`src/fanctrl/client.rs` (new, ~660 lines incl. tests) and
`src/test_support/fakes.rs` (replaced Task 2's empty stub).

**`src/fanctrl/client.rs`:**

- `PrintCommand { Speed, All }` — the only two variants; no write/set/pause path
  exists anywhere in this module. `cli_string()` maps them to the raw CLI arg
  strings sent verbatim over the socket (`--output-format JSON print speed` /
  `... print all`), per `docs/research/05-fw-fanctrl-loop.md` §"The socket".
- `FanctrlError { Absent(String), Timeout, Other(String) }` — classifies a raw
  I/O error: `ErrorKind::NotFound`/`ConnectionRefused` → `Absent`,
  `TimedOut`/`WouldBlock` → `Timeout`, anything else (including a JSON parse
  failure) → `Other`.
- `Freshness { Fresh, Stale, Absent }` and the shared `compute_freshness`
  rule (`pub(crate)`, used by both the real client and `FakeFanctrl` so the
  two can't drift): `Absent` whenever the *most recent* poll attempt could
  not even connect, overriding stamp age outright; otherwise `Fresh` only
  when `observed_at` is under 15 s old **and** `all_observed_at` is under
  90 s old (and `Some`); `Stale` otherwise, including the case where no
  view has ever been established but the last failure was not
  ENOENT/refused (e.g. a read timeout on the very first ever poll).
- `FanctrlView { strategy, active, speed_pct, temperature, ma_temperature,
  ma_interval, curve, observed_at, all_observed_at: Option<Instant> }`. A view
  is only ever *created* by a successful `All` (the only poll with enough
  data to build one); a `Speed` success only refreshes `speed_pct` and
  `observed_at` on an *existing* view, and is dropped as a no-op if no view
  exists yet. `ma_interval` is read from the **resolved strategy's own**
  `movingAverageInterval` (nested under `configuration.data.strategies`), not
  a top-level field — the fixtures only carry it there.
- `FanctrlSource` trait: `poll(cmd, now) -> Result<(), FanctrlError>`,
  `view() -> Option<&FanctrlView>`, `freshness(now) -> Freshness`. `now` is
  caller-supplied throughout (not read internally via `Instant::now()`) so
  tests drive freshness deterministically with synthetic future instants
  (`Instant::now() + Duration::from_secs(n)`, valid without sleeping) instead
  of real sleeps.
- `resolve_curve(print_all_json, strategy) -> Vec<(f64, u8)>` — exact-name
  match only (no fuzzy/case-insensitive lookup); empty on an unknown name or
  unparseable JSON.
- `UnixFanctrlClient`: one connection per command. `connect_with_timeout`
  wraps `UnixStream::connect` (which has no built-in timeout, unlike
  `TcpStream::connect_timeout`) in a thread + `mpsc` channel bounded by the
  1 s connect timeout — no new crate dependency. `send` sets a 3 s
  read/write timeout, writes the CLI string, shuts down the write half (so a
  read-to-EOF server sees the command end), reads the response to EOF.

**`src/test_support/fakes.rs`:** `FakeFanctrl` — a `VecDeque<ScriptedOutcome>`
(`All{...}`, `Speed(u8)`, `Fail(FanctrlError)`) consumed one per `poll()`
call, a `Vec<PrintCommand>` command log, and the same `compute_freshness`
rule as the real client. Polling past the end of the script panics with a
clear message (a test-authoring bug, not a runtime condition to model).

**`src/config.rs`:** added `fanctrl_socket: PathBuf`, defaulting to
`/run/fw-fanctrl/.fw-fanctrl.commands.sock` (`DEFAULT_FANCTRL_SOCKET`
const), wired into `Default for Config` and covered by a new
`fanctrl_socket_defaults_and_round_trips` test plus the existing
`roundtrip_save_load` test (extended with the new field).

**`src/fanctrl/mod.rs`:** added `pub mod client;` and updated the module doc
comment (previously said Task 7 "lands in this module later").

## TDD evidence

Followed the brief's step order. Representative red/green pairs (full detail
in the file's own step-numbered test groups):

- **Step 2/3 (parsing + resolve_curve).** RED: wrote the test module calling
  `parse_print_all`/`resolve_curve` before either function existed —
  `cargo build --tests` failed with `E0425 cannot find function`. GREEN:
  after implementing both against `tests/fixtures/fanctrl/print_all_*.json`,
  `cargo test fanctrl::client` passed (parses strategy/active/speed/
  temperature/ma_interval=60 for both quiet16 and cool16; resolve_curve
  yields the exact §Facts point lists; unknown/garbage input yields empty
  rather than panicking).
- **Step 6/7 (real socket I/O).** RED: `unix_client_read_timeout_...` and
  `unix_client_enoent_...` were written against `UnixFanctrlClient::poll`
  before `connect_with_timeout`/`classify_io_error` existed — compile
  failure. GREEN: after implementing, both pass; `unix_client_enoent_...`
  hits a real never-bound socket path (genuine ENOENT, not a mock), and
  `unix_client_read_timeout_...` spawns a real `UnixListener` that accepts
  and reads but never replies, forcing the client's 200 ms read timeout to
  actually fire (`Duration::from_millis(200)` timeouts via
  `UnixFanctrlClient::with_timeouts`, a `#[cfg(test)]`-only constructor, so
  this test doesn't wait out the real 3 s default).
- I additionally caught, during self-review (see below), that the "Speed
  refresh bumps `observed_at` but not `all_observed_at`" acceptance
  criterion was only exercised end-to-end against `FakeFanctrl`, not
  against `UnixFanctrlClient`'s own separately-implemented `poll` match arm
  for `PrintCommand::Speed`. Added
  `unix_client_speed_refresh_bumps_observed_at_but_not_all_observed_at`,
  which drives a real two-connection socket session (a `spawn_sequenced_server`
  helper accepting an `All` then a `Speed` request against the *same*
  `UnixFanctrlClient` instance) and asserts `observed_at` moves,
  `all_observed_at` doesn't, `speed_pct` updates to the new value, and the
  `All`-only fields (`strategy`, `temperature`) keep their prior value. RED
  (transiently, before I wrote the corresponding server helper): compile
  failure for the undefined `spawn_sequenced_server`. GREEN: passes, and
  reran it 5x in a loop with no flakes.

## Test results

```
cargo test
test result: ok. 512 passed; 0 failed; 2 ignored; 0 measured; 0 filtered out; finished in 2.00s
```
(The 2 ignored are pre-existing, untouched by this task. `git diff HEAD~1
HEAD -- src/fanctrl/client.rs src/test_support/fakes.rs src/config.rs |
grep -F '#[test]' | grep -c '^+'` counts 27 new `#[test]` functions added by
this commit — 512 − 27 = 485 baseline before this task, confirming none of
the 512 are stale/leftover.)

Full-suite output includes every new test passing:
`fanctrl::client::tests::*` (21 tests: parsing, resolve_curve, the two
stamps, freshness rules, real-socket ENOENT/timeout/round-trip/Speed-refresh,
the closed-PrintCommand exhaustive match), `test_support::fakes::tests::*`
(5 tests), `config::tests::fanctrl_socket_defaults_and_round_trips` (1 new
test), plus the extended `roundtrip_save_load` (21+5+1 = 27, matching the
`#[test]`-count check above).

`cargo fmt --check` on the four touched files: clean (ran `rustfmt --edition
2024` on them once).

## Quality gate: `cargo clippy --all-targets -- -D warnings` — not clean, dead_code only, precedented

46 errors, every single one `dead_code` (grepped the full output for any
`error:` line not containing "never used"/"never constructed" — zero
matches; the only other line is the final "could not compile ... due to 46
previous errors" summary). This reproduces identically on the pre-task
baseline restricted to Task 1's and Task 3's own already-landed code (21
dead_code errors via `git stash` + clippy on the unmodified branch, matching
Task 1's report exactly) — my changes add exactly 25 more (this module's own
public surface: `PrintCommand`, `FanctrlError`, `Freshness`, `FanctrlView`,
`FanctrlSource`, `compute_freshness`, the five JSON-shape structs,
`resolve_curve`, `parse_print_all`/`parse_print_speed`, `classify_io_error`,
`connect_with_timeout`, `UnixFanctrlClient` and its two associated items).

This is the same structural situation Task 1's and Task 3's reports already
documented and accepted: nothing outside `#[cfg(test)]` calls into
`fanctrl::client` yet (the sampler/poller wiring is Task 14; the controller's
consumption of `FanctrlSource` is later still), so rustc's `dead_code` lint
fires on the whole new public surface in the plain `bin` build. I did **not**
add `#[allow(dead_code)]` (forbidden by the Global Constraints) and did not
wire anything into a consumer myself (out of this task's scope). `FakeFanctrl`
contributes zero new dead_code errors — `test_support` is entirely
`#[cfg(test)]`-gated at the module level in `main.rs`, so it's invisible to
the plain-bin build clippy runs against.

## Files changed

- `src/fanctrl/client.rs` (new, ~660 lines incl. tests and doc comments)
- `src/fanctrl/mod.rs` (+`pub mod client;`, updated doc comment)
- `src/config.rs` (+`fanctrl_socket` field, default const, 2 tests)
- `src/test_support/fakes.rs` (replaced the Task 2 empty stub with `FakeFanctrl`)

## Design decisions worth a second pair of eyes

1. **`Freshness::Absent` is driven by a `last_absent` flag on the client/fake,
   not purely by the view's stamps**, even though brief step 5 says "all
   timing is computed from the view's monotonic stamps." I read that sentence
   as scoping the *Stale* rule specifically (which is purely stamp-based),
   with "Absent on ENOENT" called out separately in the same sentence as a
   non-timing override — i.e. if fw-fanctrl's socket vanishes *right now*,
   that should override even a still-numerically-fresh set of stamps, not
   just fall back to "eventually goes Stale once 15s/90s elapse." I flag this
   because it's the one place I made a judgment call between two readings of
   the same design sentence rather than following an unambiguous instruction.
2. **A `Speed` success before the first `All` is dropped (no-op), not held
   pending.** `FanctrlView`'s non-`Speed` fields (`strategy`, `curve`,
   `temperature`, ...) have no sensible value to initialize from a
   `print speed` response alone (it only ever returns `{"status", "speed"}`).
   I documented this choice in three places (the view's doc comment, and both
   `poll` implementations) rather than modeling a partial-view state, since
   nothing in the brief or design doc describes what a `Speed`-only view
   should contain. In production this is moot in practice — Task 14's poller
   is expected to issue the first `All` before any `Speed` poll — but this
   task doesn't own that ordering, so I made the client itself safe against
   the out-of-order case rather than assuming it away.
3. **`connect_with_timeout` uses a spawned thread + `mpsc::recv_timeout`**
   rather than a lower-level non-blocking connect, since `UnixStream` has no
   `connect_timeout` (unlike `TcpStream`). In practice an `AF_UNIX` connect is
   immediate either way (instant success or instant ENOENT/ECONNREFUSED), so
   this only guards a pathological case (a full accept backlog) and adds no
   measurable latency on the tested paths. If the spawned connect call itself
   never returns, the thread outlives the call (no way to cancel a blocked
   syscall from outside in std alone) — documented in the function's doc
   comment; not exercised by any test since I have no way to force a hung
   connect deterministically.

## Self-review findings

- Re-read `client.rs` end to end after the mutation pass; no leftover debug
  output, no stray `#[allow]`, no TODOs.
- Found and removed two decorative `thread::sleep(20ms)` calls in the
  socket-based tests, added under an initially-wrong assumption that
  `UnixListener::bind` happens on the background thread — it actually runs
  synchronously on the caller's thread inside `spawn_one_shot_server`, so a
  `connect()` right after that call races nothing. Verified by re-running the
  affected tests 5x in a loop after removing the sleeps: no flakes.
- Found the Speed-refresh coverage gap described above (only `FakeFanctrl`
  was exercised end-to-end for that acceptance criterion, not
  `UnixFanctrlClient`'s own separate implementation) and added the missing
  real-socket test rather than leaving it as a reported concern.
- Checked every assertion for a concrete failing case per the assertion-
  discipline instruction (see the TDD evidence section); found none that
  were decoration (each test's assertions correspond to a value the
  implementation could plausibly get wrong, e.g. field-mapping swaps,
  off-by-one on the 15s/90s boundaries, `last_absent` not clearing on a
  later success, `PrintCommand` case-sensitivity in `resolve_curve`).

## Issues or concerns

1. `cargo clippy --all-targets -- -D warnings` is not clean (dead_code only)
   — see above, expected per the deliverable's structure and precedented by
   Tasks 1 and 3 on this same branch.
2. Design decision #1 above (`Freshness::Absent` overriding stamp-based
   staleness rather than being purely stamp-derived) is the one place I'd
   most want confirmation against the original design author's intent before
   Task 16's arbiter builds mode-switching logic on top of it.

Status: **DONE_WITH_CONCERNS** (the clippy dead-code gate, expected/
precedented, and the `Freshness::Absent` reading worth double-checking —
neither blocks the task or, I believe, changes correctness).
