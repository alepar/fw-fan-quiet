## Task 11: Spike: settle the anti-windup rule

**Bead:** `fw-fanctrl-loop-9it`

**filesTouched:** `src/control/spike_antiwindup.rs`, `src/control/mod.rs`,
`docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md`

`src/control/mod.rs` — one `#[cfg(test)] pub mod spike_antiwindup;` line.
The design doc — **§2.4 only.**

### EPIC-SPECIFIC CONSTRAINT (settled outcome of three independent review rounds, not a suggestion)

> bead `fw-fanctrl-loop-9it` ("Spike: settle the anti-windup rule") is a SPIKE whose deliverable
> is a DECISION written into §2.4 of
> docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md, reached by measurement on its own
> throwaway harness. §2.4 deliberately fixes only the INVARIANTS — anti-windup is directional
> and may halt only the deepening direction, never pull u toward the measured draw; it judges
> per axis; any hold is visible — and deliberately leaves the predicate, the margins, the
> hysteresis and the GPU-HOT interaction UNSPECIFIED. Two earlier prose attempts (a
> back-calculation toward measured draw, which is a tracker rather than anti-windup; and a
> direction-blind freeze, which self-latches) were each independently confirmed Blocking.
> Therefore: (a) task `fw-fanctrl-loop-9it`'s plan section must state that the spike DECIDES
> those unspecified items by measurement and REWRITES §2.4 with the result, and must not present
> §2.4's current text as an implementable rule; (b) the plan section for `fw-fanctrl-loop-j6s`
> ("Controller loop integration"), which is blocked by 9it, must state explicitly that its
> implementer reads the REWRITTEN §2.4 from the integration branch and MUST NOT invent, infer,
> or reconstruct the anti-windup predicate, margins, hysteresis, or GPU-HOT interaction from
> prose — if §2.4 still reads as open when that task runs, that is a BLOCKED condition, not a
> licence to improvise.

**Read that again before starting.** §2.4's current text is **not an implementable rule** and
must not be treated as one. This task **decides** the predicate, the per-axis
`DEMAND_MARGIN_W`, the hysteresis/dwell/debounce, whether leaving calls `resync_error`, whether
the per-axis comparison uses the pre- or post-guard-override cap, and what a `GPU HOT` episode
does — **by measurement on the harness**, and then **rewrites §2.4** so the decided rule and its
constants are stated there as normative text. §2.4's "Open for the spike to decide and record"
list is **replaced** by the decision. The four fixed invariants stay; nothing this task decides
may violate them.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.4** (invariants), **§5** (the plant
constants), §Facts.

**This is a spike, not production code.** Its output is a decision plus a spec rewrite. The
harness is either deleted or kept as a `#[cfg(test)]` fixture — whichever the *result* argues
for; say which, and why, in your report.

### The harness

A throwaway rig around the **real** `Budget` (from Task 3): a first-order thermal plant
(tau 35, theta 20, K 0.8 per §5) **plus a demand model that decides how much of each commanded
cap is actually drawn**, so a cap can sit above the draw. Without that demand model the rig
cannot exercise the failure at all.

### The candidate rules to sweep

- no halt at all
- conditional integration halting **only the deepening direction**
- the same, with hysteresis / dwell
- per-axis versus combined

### The scenarios — every one the three roast rounds named

1. idle wind-up to the ceiling, then a load onset
2. a lull mid-session
3. a warm-start seeded from a heavier session, with a lighter load and the EC above T*
4. a mid-session target drop
5. a structurally undrawn axis (dGPU unpowered, CPU-only load)
6. a `GPU HOT` episode with the CPU at its own cap
7. oscillation around the margin when the draw sits near the cap

### Acceptance criteria (verbatim from the bead)

> every scenario above is run under every candidate and the results tabulated in the spec; the
> chosen rule holds the fan target in all of them with no self-latch and no cap-tracking;
> `DEMAND_MARGIN_W` is set per axis from the measured commanded-versus-drawn spread (RAPL
> against the slow-limit, NVML watts against the clock lock) rather than assumed, and that
> measurement is recorded in §Facts; §2.4's "open for the spike to decide" list is replaced by
> the decided rule.

### Implementation steps

1. Create `src/control/spike_antiwindup.rs` as a `#[cfg(test)]` module and declare it in
   `src/control/mod.rs`.
2. Build the rig: the real `Budget`, the first-order plant at tau 35 / theta 20 / K 0.8, and the
   demand model. Seeded RNG only — every run must be reproducible.
3. Encode the seven scenarios as data, and the four candidate rules as a small trait or enum, so
   the sweep is a cross product rather than seven hand-written cases per candidate.
4. **Measure `DEMAND_MARGIN_W` per axis on the real machine** — RAPL against the slow-limit for
   the CPU, NVML watts against the clock lock for the GPU. Record the measurement (numbers,
   method, date) in **§Facts**. Do **not** assume a value; if the machine measurement is
   genuinely unavailable, that is a BLOCKED condition to report, not a number to invent.
5. Run the full sweep. Tabulate candidate x scenario in the spec, with the criterion applied to
   each cell: does the fan target hold, is there a self-latch, is there cap-tracking?
6. Pick the winner. Verify it against the four fixed invariants explicitly — directional; never
   pulls `u` toward the draw; per-axis; the hold is visible.
7. **Rewrite §2.4**: replace the "Open for the spike to decide and record" paragraph with the
   decided rule stated normatively — the predicate, the per-axis `DEMAND_MARGIN_W`, the
   hysteresis/dwell/debounce, whether leaving calls `resync_error`, pre- or post-guard-override
   cap, and the `GPU HOT` interaction. Keep the four invariants and the "why prose failed"
   history; a reader arriving at §2.4 after this task must find an **implementable** rule, and
   must not be able to mistake the old open list for one.
8. Decide the harness's fate (delete, or keep as a `cfg(test)` fixture) and act on it.
9. Run the test suite and the linter; both clean.

### Deliverable

A rewritten §2.4 that states an implementable, measured anti-windup rule and its constants, plus
the sweep table that justifies it, plus the §Facts entry recording the `DEMAND_MARGIN_W`
measurement. Report the chosen rule and its constants **in your report text** as well, so the
coordinator can see the decision without opening the spec.

---

