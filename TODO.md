# bazerame-fans TODO

The implemented controller follows [Per-device temperature loops, revision 4](docs/superpowers/runs/2026-09-11-per-device-temperature-loops/2026-09-11-per-device-temperature-loops-design.md). Older milestone plans under `docs/plans/` and historical specifications describe retired implementations and remain as project history.

## Completed

- Read-only monitoring, telemetry, and the terminal dashboard
- Manual CPU/GPU actuation with startup, shutdown, panic, and signal restoration
- Sensor and actuator self-test
- fw-fanctrl read-only socket polling and EC reconciliation
- Shared T* source with independent CPU-watt and GPU-clock temperature loops
- Per-device shadow candidates, hot-guard maximum ratchets, and paired verification
- Native per-device step calibration with keyed gains, paired warm starts, and qualified T* persistence
- Schema-v3 telemetry and the revision-4 TUI
- Deterministic offline unit, integration, and plant acceptance suites

## Hardware validation

- [ ] Run the guided per-device calibration on the target Framework 16
- [ ] Run the 30-minute gaming acceptance session
- [ ] Validate the VR/VRAM label spike on physical hardware
