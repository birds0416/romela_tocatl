# TOCATL Controller Audit and Roadmap

## What exists

- 22 recovered experimental scripts plus the deployed systemd unit.
- The deployed walking controller directly configures `can0` and `can1`, creates
  all 18 CyberGear objects, streams MIT-style commands, and reads position,
  velocity, and torque feedback.
- Temperature is available in the chosen CyberGear driver but is not surfaced
  by the deployed walking UI or used for protection.
- Several scripts automatically enable motors and call `set_zero_position()` at
  startup. The driver documents that this zero is lost at power-down.
- The old service used `Restart=always`, so a process-exiting software stop could
  be followed by an automatic restart.
- `MantisModev1.py` and `Motion with Controller All Legs.py` are byte-identical;
  `IK Testing.py` is empty. The recovered folder is valuable evidence, but it is
  not maintainable production architecture.

## Recommended sequence

### 1. Observation and mapping

- Verify each physical motor label, joint direction, ID, and CAN branch while
  motor power is isolated.
- Add a reviewed telemetry-only tool for position, velocity, torque estimate,
  temperature, bus voltage, mode, warning bits, and fault bits. Initial scaffold:
  `tocatl_diagnostics.cybergear_telemetry`, backed by
  `tocatl_control.cybergear_adapter`.
- Log timestamp, motor ID, freshness, and CAN error counters to CSV or rosbag.

### 2. Safe startup state machine

Use explicit states: `DISARMED -> SELF_TEST -> CALIBRATION_CHECK -> READY -> ARMED
-> FAULT`. Startup must remain disarmed. Arming should require an intentional
operator action, current telemetry from all expected motors, valid calibration,
and no active faults. A fault should latch until reviewed and reset.

### 3. Calibration and zero position

- Remove all automatic zero writes.
- Define a repeatable mechanical reference pose and conservative joint windows.
- Store per-joint offsets, signs, limits, date, method, and robot revision in a
  versioned calibration file. Initial validator: `tocatl_control.calibration`;
  template: `config/calibration-template.json`.
- Make calibration a separate command with confirmation and a dry-run report.
- After any zero operation, power-cycle and verify behavior before motion tests.

### 4. Telemetry and protection

- Separate the fast command loop from a lower-rate health/telemetry loop.
- Detect missing/stale feedback, over-temperature, undervoltage, motor faults,
  CAN bus-off/errors, commands outside limits, and controller-heartbeat loss.
- Route future one-motor motion through `tocatl_control.command_limiter` before
  any driver command call.
- Add warning, derating, and fault thresholds only after confirming protocol
  units and collecting unloaded/loaded baseline data.
- Treat reported torque as an actuator estimate until calibrated against known
  loads; it is useful for trends, contact inference, and overload detection but
  is not automatically a laboratory-grade force measurement.

### 5. Better control and gait structure

- Move motor I/O, safety state, kinematics, gait generation, input mapping, and
  logging into separate modules with tests.
- Start with a phase-based wave or ripple gait at low duty factor and speed.
- Add smooth gait transitions, body-height/attitude commands, per-leg workspace
  checks, velocity/acceleration limits, and stance/swing interpolation.
- Then add contact/load inference from torque and velocity, adaptive foot
  touchdown, support-polygon checks, thermal/load-aware speed derating, and
  recovery behavior.
- Keep Spider Art choreography above the locomotion layer: named, bounded poses
  and motions should call a safe controller rather than directly command motors.

## Hardware test ladder

1. Offline tests and static inventory.
2. Pi powered, motor rail off: interface and configuration checks.
3. Passive CAN observation, if traffic already exists.
4. Active telemetry requests with robot supported and motion commands absent.
5. One spare motor or one isolated joint at conservative limits.
6. One suspended leg, then all legs with the body mechanically supported.
7. Low-energy stance and gait tests with a physical motor-power E-stop.

Do not advance stages merely because software runs; record an explicit pass/fail
result and operator approval at each stage.
