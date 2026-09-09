## Task 14: Sample plumbing + socket poller

**Bead:** `fw-fanctrl-loop-sov`

**filesTouched:** `src/types.rs`, `src/sensors/sampler.rs`, `src/sensors/mod.rs`, `src/main.rs`

`src/main.rs` — poller construction and shutdown join only.

### Global constraints

All of "Global Constraints" above applies. Normative: **§3.4**, §2.1, §2.2.

### The three-thread rule — the reason this task exists in this shape

- The **sampler** ticks at 1 Hz, reads EC and AC each tick, and merges the latest view.
- The **`FanctrlPoller`** runs on its own thread: `print speed` every 5 s, `print all` every
  30 s, **never faster**, sharing an `Arc<Mutex<...>>`.
- **The NVMe temperature is polled at 30 s on a thread of its own — neither the sampler tick nor
  the `FanctrlPoller`.** §3.4: a SMART admin read can block for the kernel's 60 s
  `admin_timeout`. On the sampler it would stall the control loop; on the poller it would fake a
  socket outage through the 15 s `print speed` staleness rule and drop the loop out of Mode A.
  It publishes a stamped last-good value; a missing or stale value surfaces as `None`.

**All freshness and cadence decisions are computed from the `Sample` / `FanctrlView` monotonic
timestamps.** The poller's own `Instant` drives only its cadence — never a freshness verdict.
No wall-clock anywhere in this path.

### What this task owns

`Sample` gains `ec: Option<EcReading>`, `ec_valid`, `nvme_temp_c: Option<f64>`,
`fanctrl: Option<FanctrlView>`, `fanctrl_freshness`, `fanctrl_view_changed: bool` (**true on the
first sample whose view `all_observed_at` differs from the previous sample's — a speed-only
refresh never sets it**), `on_ac`, and `resumed` (already produced by the existing resume
handler, now also consumed downstream). Plus the `FanctrlPoller` cadence, poller construction
from `Config::fanctrl_socket` with the real `UnixFanctrlClient`, and the shutdown join.

### Acceptance criteria (verbatim from the bead)

> with the fake source, over a 60 s scripted run the poller issues `Speed` at 5 s +/-1 tick and
> `All` exactly twice; `Stale` after 90 s of `All` failures and after 15 s of `Speed` failures;
> `fanctrl_view_changed` is true exactly once per new `All` view and never on a `Speed`-only
> refresh; **a scripted NVMe read that blocks for 60 s delays no `Sample`, leaves `Freshness`
> `Fresh` and the `print speed` cadence intact, and surfaces only as `nvme_temp_c: None`**; the
> real client is constructed from the configured path and both threads join on shutdown.

### Implementation steps (TDD)

1. **Test first:** a `Sample` carries every new field. Then extend `Sample` in `src/types.rs`.
2. **Test first:** over a 60 s scripted run against `FakeFanctrl`, the poller's command log shows
   `Speed` every 5 s (+/-1 tick) and `All` exactly twice. Then implement `FanctrlPoller` and its
   cadence.
3. **Test first:** `Freshness` becomes `Stale` after 90 s of `All` failures, and after 15 s of
   `Speed` failures. Then wire the freshness computation off the view's monotonic stamps.
4. **Test first:** `fanctrl_view_changed` is true on exactly the first sample after a new `All`
   view and **never** after a `Speed`-only refresh. Then implement.
5. **Test first — the NVMe isolation test, the one that justifies the third thread:** a scripted
   NVMe read that blocks for 60 s delays **no** `Sample`, leaves `Freshness` `Fresh`, leaves the
   `print speed` cadence intact, and surfaces only as `nvme_temp_c: None`. Then implement the
   30 s NVMe thread with its stamped last-good value.
6. **Test first:** the real `UnixFanctrlClient` is constructed from `Config::fanctrl_socket`, and
   both background threads join on shutdown. Then wire `src/main.rs`.
7. **Verify:** no wall-clock (`SystemTime::now`) appears in any freshness or cadence decision on
   this path. Record the search in your report.
8. Run the test suite and the linter; both clean.

### Deliverable

A 1 Hz `Sample` carrying the full new field set, fed by two independent background threads whose
blocking behaviour cannot reach the control loop.

---

