## Task 7: fw-fanctrl socket client

**Bead:** `fw-fanctrl-loop-58u`

**filesTouched:** `src/fanctrl/client.rs`, `src/fanctrl/mod.rs`, `src/config.rs`,
`src/test_support/fakes.rs`

`src/fanctrl/mod.rs` — mod declaration only.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.1**, §3.4, §Facts (the two live
curves and `movingAverageInterval` 60).

**The read-only invariant is this task's to enforce structurally.** `PrintCommand { Speed, All }`
is the **only** command type the client can send. There is no escape hatch, no raw-string send,
no `set`/`use`/`pause` variant. A later task must not be able to write a fan speed through this
type even by mistake.

### What this task owns

`PrintCommand`, the `FanctrlSource` trait, `UnixFanctrlClient` (connect 1 s, read 3 s, raw CLI
string, read to EOF), `FakeFanctrl` in `src/test_support/fakes.rs` recording every command and
replaying scripted views/failures, `FanctrlView` (§2.1) with **two stamps** — `observed_at` (any
poll) and `all_observed_at` (last `print all`) — `curve` as the raw `Vec<(f64, u8)>` from
`resolve_curve(print_all_json, strategy)` (exact-name match in `strategies`),
`Freshness { Fresh, Stale, Absent }` with the **15 s (`print speed`) / 90 s (`print all`)**
rules, and the config key `fanctrl_socket` (default `/run/fw-fanctrl/.fw-fanctrl.commands.sock`).

### Acceptance criteria (verbatim from the bead)

> parses the fixtures (strategy, active, speed, temperature, ma interval); `resolve_curve` yields
> exactly the §Facts points for `quiet16` and `cool16`; ENOENT -> Absent; read timeout -> Stale;
> `print speed` failing for 15 s while `print all` is fresh -> Stale; a `print speed` refresh
> bumps `observed_at` but not `all_observed_at`; the fake's command log contains only
> `Speed`/`All`.

### Implementation steps (TDD)

1. Create `src/fanctrl/client.rs` and declare it in `src/fanctrl/mod.rs`.
2. **Test first:** parsing `tests/fixtures/fanctrl/print_all_quiet16.json` yields the strategy
   name, `active`, `speed`, `temperature` and `movingAverageInterval` 60. Then implement
   `FanctrlView` and the parser.
3. **Test first:** `resolve_curve(print_all_json, "quiet16")` yields exactly the §Facts quiet16
   points, and the same for `cool16`; an unknown strategy name yields no curve. Match the
   strategy name **exactly** — no fuzzy or case-insensitive matching. Then implement.
4. **Test first:** the two stamps — a `print speed` refresh bumps `observed_at` and leaves
   `all_observed_at` untouched; a `print all` bumps both. Then implement.
5. **Test first:** `Freshness` — `Fresh` inside both windows; `Stale` when the last `print all`
   is older than 90 s; `Stale` when `print speed` has been failing for 15 s **even while
   `print all` is fresh**; `Absent` on ENOENT. All timing is computed from the view's monotonic
   stamps. Then implement.
6. **Test first:** a read timeout maps to `Stale`, not `Absent`. Then implement the 1 s connect
   / 3 s read timeouts in `UnixFanctrlClient`.
7. **Test first:** `PrintCommand` has exactly two variants and `FanctrlSource` accepts nothing
   else; `FakeFanctrl`'s command log after a scripted session contains only `Speed`/`All`. Then
   implement `FakeFanctrl` in `src/test_support/fakes.rs` (replacing the empty stub from Task 2)
   with scripted views and scripted failures.
8. **Test first:** `Config` round-trips `fanctrl_socket` with the documented default. Then add
   the key.
9. Run the test suite and the linter; both clean.

### Deliverable

A read-only socket client plus a scriptable fake, both unit-tested on the Task 2 fixtures.

---

