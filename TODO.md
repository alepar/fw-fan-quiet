# bazerame-fans TODO

Plan: `docs/plans/2026-07-03-bazerame-fans-plan.md` (design: `docs/plans/2026-07-03-bazerame-fans-design.md`)

Rules: one task at a time, in a subagent, TDD, commit per task, tick the box when the
task's commit lands. Human-gated items are marked 👤.

## Milestone 1 — Monitor (read-only dashboard)
- [x] 1. Project scaffold (cargo + deps)
- [x] 2. Core types + ring buffer
- [x] 3. RAPL power sensor (wraparound-safe)
- [x] 4. hwmon sensors (fans, Tctl, amdgpu)
- [x] 5. CPU utilization + frequency sensors
- [x] 6. NVML sensor wrapper
- [x] 7. Sampler thread + event enum
- [x] 7b. Telemetry JSONL logger (samples + decisions, for offline controller review)
- [x] 8. UI Model + update()
- [x] 9. Dashboard view + main wiring 👤 (visual check of live dashboard)

## Milestone 2 — Manual actuation + safety
- [x] 10. Command runner trait + ryzen_smu module handling
- [x] 11. CPU actuator (ryzenadj) + stock restore
- [x] 12. GPU actuator (NVML clock locks)
- [x] 13. Restore guard + startup reset
- [x] 14. Controller thread + ControlStatus
- [x] 15. Manual-mode UI
- [x] 16. Selftest subcommand + M2 hardware verification

## Milestone 3 — Calibration
- [x] 17. CPU burner (implemented with task 16; selftest uses it)
- [ ] 18. Steady-state detector
- [ ] 19. Clock→watts LUT sweep
- [ ] 20. Thermal model fit + RLS
- [ ] 21. Config + state persistence
- [ ] 22. Calibration runner + UI wizard 👤 (real ~30-min calibration run)

## Milestone 4 — Closed loop
- [ ] 23. Demand estimator + allocator
- [ ] 24. GPU watts→clock inner PI
- [ ] 25. Auto mode wiring + UI 👤 (real gaming session validation)

## Milestone 5 — Adaptive & hardening
- [ ] 26. Bounded trim integrator
- [ ] 27. Online RLS + trust monitor
- [ ] 28. Watchdogs + emergency release
- [ ] 29. Resume hardening + polish + README 👤 (acceptance + abuse tests)
