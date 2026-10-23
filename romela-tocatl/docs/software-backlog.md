# TOCATL Software Backlog

This is the working software plan for Spider Art and the later Spider Robot V2.
The theme is boring on purpose: make every risky thing explicit, logged, and
hard to trigger by accident.

## Current baseline

- The recovered deployed controller is preserved under `legacy/deployed-2026-03-15`.
- `tocatl.service` on the Pi is disabled and should stay disabled until the new
  startup path is reviewed.
- Active code now contains offline-safe inventory tools and early ROS 2 package
  scaffolding.
- The canonical motor map is in `tocatl_diagnostics.motor_map`.
- The first pure safety/arming rules live in `tocatl_control.safety_state`.
- The first telemetry-only helper lives in
  `tocatl_diagnostics.cybergear_telemetry`.
- The first calibration schema/validator lives in `tocatl_control.calibration`;
  the checked-in template is intentionally incomplete.
- `tocatl_control.cybergear_adapter` is the stable boundary between TOCATL logic
  and the selected CyberGear Python driver.
- `tocatl_control.command_limiter` is the pure software gate before any future
  one-motor command sender.
- `tocatl_kinematics.legacy_geometry` and `tocatl_gait.legacy_reference` contain
  pure reference constants and math extracted from the recovered controller.
- `scripts/pi_dry_run_checks.sh` is the no-live-CAN software gate to run on the
  Pi before any live telemetry session.

## Near-term software changes

| Priority | Change | Why it matters | First useful output |
| --- | --- | --- | --- |
| 1 | Telemetry-only CyberGear tool | Learn motor state without motion commands | CSV rows for position, velocity, torque estimate, temperature, bus voltage, mode, faults |
| 2 | Calibration file format | Stop treating volatile `set_zero_position()` as persistent truth | YAML/JSON with motor ID, joint name, sign, offset, limits, date, method |
| 3 | Gated startup / arming | Prevent boot-time enabling, zeroing, or restart surprises | State machine integrated into ROS node |
| 4 | Motor adapter wrapper | Keep robot code independent from one CyberGear Python library | `CyberGearAdapter` interface with read-only first implementation |
| 5 | One-motor command limiter | Bound speed, torque/current, position window, step size | Pure limiter exists; next is a reviewed one-joint motion script |
| 6 | Logging | Make current, temperature, and faults reviewable after tests | CSV first; rosbag later |
| 7 | Gait layer | Replace ad-hoc walking scripts with named bounded behaviors | Legacy wave/ripple/tripod constants are extracted; next is a slow stand/park cue planner |

## Legacy reference extraction

The recovered `WalkingExcursion_v1.py` script is now mined for non-hardware
reference material only:

- `tocatl_kinematics.legacy_geometry`: link lengths, body reference, legacy
  FK/IK branch, and old theta/motor transforms.
- `tocatl_gait.legacy_reference`: wave/ripple/tripod phase offsets and old
  walking-envelope constants.

These modules do not import CAN, ROS, joystick, or CyberGear driver code. They
are useful for simulation, documentation, calibration planning and future
Spider Art cue generation, but they must not bypass the safety state,
calibration file, command limiter or physical motor-power stop path. More detail
lives in [`docs/legacy-controller-reference.md`](legacy-controller-reference.md).

## Telemetry helper

Before live telemetry, run:

```bash
bash scripts/pi_dry_run_checks.sh
```

Dry-run on any computer:

```bash
PYTHONPATH=src/tocatl_control:src/tocatl_diagnostics \
  python3 -m tocatl_diagnostics.cybergear_telemetry
```

After the protected harness/no-motion gate passes on the Pi, live one-bus read:

```bash
PYTHONPATH=src/tocatl_control:src/tocatl_diagnostics \
  python3 -m tocatl_diagnostics.cybergear_telemetry \
  --live --interface socketcan --channel can0 --bus can0 \
  --csv telemetry-can0.csv
```

Then repeat for `can1`. Live mode sends CAN read requests only:

- `request_motor_state`
- `request_motor_fault_status`

It does not enable motors, write zero position, set run mode, set parameters, or
send motion commands. It still must not be treated as passive observation.

The telemetry helper uses `tocatl_control.cybergear_adapter` rather than calling
driver objects directly. Future command limiting and ROS nodes should use that
same adapter boundary.

## Calibration file

Generate a template:

```bash
PYTHONPATH=src/tocatl_control python3 -m tocatl_control.calibration \
  --write-template config/my-calibration.json
```

Validate it:

```bash
PYTHONPATH=src/tocatl_control python3 -m tocatl_control.calibration \
  config/my-calibration.json
```

Required per joint:

- `motor_id`, `leg`, `joint`, `bus`
- `sign`: `1` or `-1`
- `zero_offset_rad`
- `min_position_rad`
- `max_position_rad`
- `park_position_rad`
- `method`

The template has all 18 joints but null calibration values, so it fails
validation by design. A calibration file should only become an arming input after
every joint has a supervised reference method, conservative limits, and a park
pose compatible with the physical robot.

## Command limiter

`tocatl_control.command_limiter` accepts a desired joint-frame command and either
returns a driver-facing bounded command or rejects with reasons. It requires:

- safety state `ARMED`
- matching motor ID across request, calibration and telemetry
- fresh telemetry with no motor faults
- complete calibration
- requested position inside calibrated joint limits
- requested step inside the configured single-motor step envelope

Current default early-test envelope:

- velocity: 2 output rpm, converted to approximately `0.209 rad/s`
- torque: `0.5 Nm`
- single command position step: `0.10 rad`

The defaults are conservative placeholders for software bring-up. They are not
museum-performance limits and should be revised with measured telemetry,
mechanical support, and electrical review.

## Driver direction

`jgillick/python-cybergear` is useful as a reference and possible dependency
because it exposes motor state requests, parameter reads, fault requests, and
the warning that zero position is lost after power-down. Do not tie the robot
architecture directly to that library. `tocatl_control.cybergear_adapter`
normalizes driver state into `MotorTelemetry` and keeps read-only requests
separate from future motion APIs.

## ROS 2 mental model

ROS 2 is the message bus and launch system. It should not be the safety system.

- Nodes are small processes, for example: telemetry reader, safety state,
  calibration manager, gait generator, command sender, logger.
- Topics are streams, for example `/joint_commands`, `/motor_states`,
  `/robot_state`, `/faults`.
- Services/actions are deliberate operations, for example arm, disarm, calibrate,
  park, run named Spider Art cue.
- Launch files start the set of nodes with parameters.

The physical E-stop/contactors must work even if ROS, Python, Linux, Wi-Fi, or
the Xbox controller fails.

## Refactor direction

Avoid one giant `walking.py`. Split responsibilities:

```text
tocatl_diagnostics  -> read-only inventory and test helpers
tocatl_control      -> safety state, arming, command limits, motor adapter
tocatl_kinematics   -> leg geometry, IK/FK, joint limits
tocatl_gait         -> foot trajectories and named behaviors
tocatl_bringup      -> launch/config files
```

Each layer should be testable without robot hardware wherever possible.
