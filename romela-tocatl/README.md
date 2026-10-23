# TOCATL Hexapod

Private recovery and redevelopment repository for the RoMeLa TOCATL spider
platform. The recovered March 2026 deployment is preserved under `legacy/`; new
work should happen in the ROS 2 packages under `src/`.

## Start Here

- [`docs/motor-map.md`](docs/motor-map.md) - canonical 18-motor / two-CAN-bus map.
- [`docs/pi-usb-inventory.md`](docs/pi-usb-inventory.md) - Pi and USB/CAN inventory.
- [`docs/pi-dry-run-checks.md`](docs/pi-dry-run-checks.md) - safe Pi software checks before live CAN.
- [`docs/controller-roadmap.md`](docs/controller-roadmap.md) - controller audit and safety direction.
- [`docs/software-backlog.md`](docs/software-backlog.md) - current software backlog and ROS 2 mental model.
- [`docs/github-workflow.md`](docs/github-workflow.md) - repo workflow.

## Current Safety Rule

Do not run legacy walking scripts as ordinary inspection tools. The old deployed
service enabled motors and wrote volatile zero positions during startup. New
software must start disarmed, require fresh telemetry and valid calibration, and
need an explicit operator arm action before motion.

The physical motor-power E-stop/contacting system remains independent of Linux,
ROS, Python, Wi-Fi, and the Xbox controller.

## Python Virtual Environment Setup

This project uses a Python virtual environment to manage dependencies separately from system Python packages.

### Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy installed
- Python 3.10+

---

## Initial Setup

### 1. Create Python Virtual Environment
```bash
# Navigate to project root
cd ~/romela_tocatl

# Create virtual environment
python3 -m venv .venv

# Activate virtual environment
source .venv/bin/activate
```

### 2. Install Python Dependencies
```bash
# Upgrade pip
pip install --upgrade pip

# Install core dependencies
pip install numpy scipy matplotlib

# Install ROS2 Python packages (if needed beyond ros-jazzy-desktop)
pip install transforms3d

# For development/testing
pip install pytest black flake8
```

## Run Tests

From the repository root:

```bash
python3 -m unittest discover -s test -v
```

For the full no-live-CAN dry-run gate:

```bash
bash scripts/pi_dry_run_checks.sh
```

Use `--skip-service-check` only for laptop verification where `systemctl --user`
is unavailable.

The same non-hardware checks run in GitHub Actions on PRs and pushes via
[software-safety.yml](.github/workflows/software-safety.yml).

## Telemetry Dry Run

Before touching live CAN, confirm the telemetry helper's schema and motor list:

```bash
PYTHONPATH=src/tocatl_control:src/tocatl_diagnostics \
  python3 -m tocatl_diagnostics.cybergear_telemetry
```

Live telemetry is intentionally gated behind `--live`; it sends read requests
and should only be used after the physical no-motion gate passes.

## Calibration Template

Generate an incomplete calibration file:

```bash
PYTHONPATH=src/tocatl_control python3 -m tocatl_control.calibration \
  --write-template config/my-calibration.json
```

Validate it:

```bash
PYTHONPATH=src/tocatl_control python3 -m tocatl_control.calibration \
  config/my-calibration.json
```

The checked-in [config/calibration-template.json](config/calibration-template.json)
is intentionally invalid for arming until real signs, offsets, park positions,
and conservative joint limits are filled from a supervised procedure.

## Command Limiter

[tocatl_control.command_limiter](src/tocatl_control/tocatl_control/command_limiter.py)
is the pure software gate before any future motor command sender. It requires:

- `SafetyState.ARMED`
- matching motor ID across request, calibration, and telemetry
- fresh telemetry with no active motor faults
- complete calibration
- requested position inside calibrated joint limits
- requested step inside the current single-motor step limit

It rejects unsafe position requests instead of silently clamping them. Velocity
and torque are bounded to the configured early-test envelope.
