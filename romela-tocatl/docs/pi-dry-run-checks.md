# Pi Dry-Run Checks

Use this before any live CAN telemetry or motion testing. It verifies the current
software scaffold without enabling motors, zeroing motors, or sending live
CyberGear requests.

## Physical conditions

- Pi powered from a known-good USB-C supply.
- `tocatl.service` expected to be disabled and inactive.
- Do not run recovered legacy scripts.
- Do not pass `--live` to telemetry tools.
- Motor rail should remain disconnected unless a reviewed hardware test says
  otherwise.

## Run on the Pi

From the checked-out repository:

```bash
cd ~/Desktop/dev/romela_tocatl
git fetch origin
git switch codex/software-safety-backlog
git pull --ff-only origin codex/software-safety-backlog
bash scripts/pi_dry_run_checks.sh
```

The script checks:

- legacy `tocatl.service` state
- unit tests
- motor map dry-run output
- telemetry schema dry-run output
- calibration template fails validation by design

## Local laptop check

For laptop-only verification where `systemctl --user` is unavailable:

```bash
bash scripts/pi_dry_run_checks.sh --skip-service-check
```

Only use `--skip-service-check` away from the Pi. On the robot computer, keep the
service check enabled.

## Pass criteria

- Script exits with status `0`.
- No service-state failure.
- Unit tests pass.
- Telemetry output shows only dry-run schema/motor inventory.
- Calibration template is reported invalid.

Passing this script does not authorize live CAN or motion. It only proves the
software scaffold can be imported and exercised safely.
