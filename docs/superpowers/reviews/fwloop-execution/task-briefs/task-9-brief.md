## Task 9: Actuator read-back

**Bead:** `fw-fanctrl-loop-jpg`

**filesTouched:** `src/actuators/cpu.rs`, `src/actuators/gpu.rs`, `src/control/controller.rs`

`src/control/controller.rs` — **the two actuator call sites only.** The RAPL stickiness watchdog
in `on_sample` is **untouched**.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.9**, §Facts (`ryzenadj --info` fails
while `ryzen_smu` is loaded; `nvidia-smi` reports `power.limit` N/A, so the GPU read-back is the
measured SM clock under load).

### What this task owns

`src/actuators/cpu.rs`: write `--slow-limit --stapm-limit --fast-limit`, then run
`ryzenadj --info` through the existing `Runner`, parse the `PPT LIMIT SLOW`, `PPT LIMIT FAST`
and `STAPM LIMIT` rows, and return
`WriteVerdict { Verified(w), Mismatch { field, commanded, read }, Unreadable, Unverifiable }`.
Slow and fast must agree within **0.5 W**; **STAPM is not required**; a failed `--info` yields
**`Unreadable`, never `Mismatch`** (that distinction is what stops a module-load precondition
from being read as a hardware fault).

`src/actuators/gpu.rs`: `verify_lock(gpu_util, gpu_sm_mhz) -> WriteVerdict` using the LUT-sweep
pin rule — util above 90 %, sm at most locked + 30, over 3 samples.

Existing controller callers compile by treating any non-`Verified` verdict as today's
success/failure until Task 19.

### Acceptance criteria (verbatim from the bead)

> parser on the fixture table; mismatch detected on a fake runner returning a stale table; a
> runner whose `--info` fails yields `Unreadable`; GPU verdict `Unverifiable` under 90 % util and
> `Mismatch` when the pinned clock exceeds lock + 30 for 3 samples; the existing RAPL watchdog
> tests still pass.

### Implementation steps (TDD)

1. **Test first:** the `ryzenadj --info` parser on `tests/fixtures/ryzenadj_info.txt` extracts
   the three named rows as watts. Then implement the parser.
2. **Test first:** commanding a value the fixture table agrees with (within 0.5 W on slow and
   fast) yields `Verified(w)`; STAPM disagreeing does **not** break verification. Then implement
   `set_sustained_mw`'s new return type.
3. **Test first:** a fake `Runner` returning a stale table yields
   `Mismatch { field, commanded, read }` naming the offending field. Then implement.
4. **Test first:** a fake `Runner` whose `--info` invocation fails yields **`Unreadable`** — and
   assert explicitly that it is not `Mismatch`. Then implement.
5. **Test first:** `verify_lock` returns `Unverifiable` when util is below 90 %; `Verified` when
   util is above 90 % and sm is at most lock + 30; `Mismatch` when the pinned clock exceeds
   lock + 30 across 3 samples (and not on 1 or 2). Then implement.
6. Update the two controller call sites to compile against `WriteVerdict`, mapping non-`Verified`
   to today's behaviour with a comment naming `fw-fanctrl-loop-j6s`. Do not touch the RAPL
   stickiness watchdog.
7. Run the test suite and the linter; both clean, **including the pre-existing RAPL watchdog
   tests**.

### Deliverable

Both actuators return a `WriteVerdict`, unit-tested on the fixture and on fake runners, with the
controller still behaving as before.

---

