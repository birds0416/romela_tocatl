from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).parents[1] / "src" / "tocatl_control"
sys.path.insert(0, str(PACKAGE_ROOT))

from tocatl_control.calibration import JointCalibration  # noqa: E402
from tocatl_control.command_limiter import (  # noqa: E402
    CommandLimits,
    MotorCommandRequest,
    limit_motor_command,
)
from tocatl_control.cybergear_adapter import MotorTelemetry  # noqa: E402
from tocatl_control.safety_state import SafetyState  # noqa: E402


def calibration(**overrides):
    data = {
        "motor_id": 11,
        "leg": 1,
        "joint": "hip_yaw",
        "bus": "can1",
        "sign": 1,
        "zero_offset_rad": 0.2,
        "min_position_rad": -0.5,
        "max_position_rad": 0.5,
        "park_position_rad": 0.0,
        "method": "unit test",
        "notes": "",
    }
    data.update(overrides)
    return JointCalibration(**data)


def telemetry(**overrides):
    data = {
        "timestamp_s": 1.0,
        "motor_id": 11,
        "fresh": True,
        "position_rad": 0.0,
        "velocity_rad_s": 0.0,
        "torque_estimate_nm": 0.0,
        "temperature_c": 30.0,
        "bus_voltage_v": 24.0,
        "mode": "standby",
        "faults": (),
    }
    data.update(overrides)
    return MotorTelemetry(**data)


class CommandLimiterTests(unittest.TestCase):
    def test_accepts_safe_command_and_applies_calibration_transform(self):
        decision = limit_motor_command(
            MotorCommandRequest(motor_id=11, position_rad=0.05, velocity_rad_s=0.1, torque_nm=0.2),
            calibration(),
            telemetry(position_rad=0.0),
            SafetyState.ARMED,
        )
        self.assertTrue(decision.accepted)
        self.assertIsNotNone(decision.command)
        self.assertEqual(decision.command.motor_id, 11)
        self.assertAlmostEqual(decision.command.motor_position_rad, 0.25)
        self.assertAlmostEqual(decision.command.motor_velocity_rad_s, 0.1)
        self.assertAlmostEqual(decision.command.torque_nm, 0.2)

    def test_rejects_when_not_armed(self):
        decision = limit_motor_command(
            MotorCommandRequest(motor_id=11, position_rad=0.0),
            calibration(),
            telemetry(),
            SafetyState.READY,
        )
        self.assertFalse(decision.accepted)
        self.assertIn("safety state is READY, not ARMED", decision.reasons)

    def test_rejects_outside_position_limits_instead_of_clamping(self):
        decision = limit_motor_command(
            MotorCommandRequest(motor_id=11, position_rad=0.6),
            calibration(),
            telemetry(position_rad=0.49),
            SafetyState.ARMED,
        )
        self.assertFalse(decision.accepted)
        self.assertIn("outside", decision.reasons[0])

    def test_rejects_large_position_step(self):
        decision = limit_motor_command(
            MotorCommandRequest(motor_id=11, position_rad=0.2),
            calibration(),
            telemetry(position_rad=0.0),
            SafetyState.ARMED,
            CommandLimits(max_position_step_rad=0.05),
        )
        self.assertFalse(decision.accepted)
        self.assertIn("requested step", decision.reasons[0])

    def test_clamps_velocity_and_torque(self):
        decision = limit_motor_command(
            MotorCommandRequest(motor_id=11, position_rad=0.01, velocity_rad_s=9.0, torque_nm=-9.0),
            calibration(),
            telemetry(position_rad=0.0),
            SafetyState.ARMED,
            CommandLimits(max_abs_velocity_rad_s=0.2, max_abs_torque_nm=0.3, max_position_step_rad=0.1),
        )
        self.assertTrue(decision.accepted)
        self.assertAlmostEqual(decision.command.motor_velocity_rad_s, 0.2)
        self.assertAlmostEqual(decision.command.torque_nm, -0.3)

    def test_negative_sign_flips_driver_position_and_velocity(self):
        decision = limit_motor_command(
            MotorCommandRequest(motor_id=11, position_rad=0.04, velocity_rad_s=0.1, torque_nm=0.0),
            calibration(sign=-1, zero_offset_rad=0.2),
            telemetry(position_rad=0.0),
            SafetyState.ARMED,
        )
        self.assertTrue(decision.accepted)
        self.assertAlmostEqual(decision.command.motor_position_rad, -0.24)
        self.assertAlmostEqual(decision.command.motor_velocity_rad_s, -0.1)

    def test_rejects_stale_telemetry_and_faults(self):
        decision = limit_motor_command(
            MotorCommandRequest(motor_id=11, position_rad=0.0),
            calibration(),
            telemetry(fresh=False, faults=("encoder",)),
            SafetyState.ARMED,
        )
        self.assertFalse(decision.accepted)
        self.assertIn("telemetry is stale or missing", decision.reasons)
        self.assertIn("active motor faults: encoder", decision.reasons)

    def test_rejects_incomplete_calibration(self):
        decision = limit_motor_command(
            MotorCommandRequest(motor_id=11, position_rad=0.0),
            calibration(sign=None),
            telemetry(),
            SafetyState.ARMED,
        )
        self.assertFalse(decision.accepted)
        self.assertIn("calibration sign must be -1 or 1", decision.reasons)

    def test_module_has_no_driver_or_motion_api(self):
        source = (PACKAGE_ROOT / "tocatl_control" / "command_limiter.py").read_text()
        forbidden = ("import can", ".send(", ".enable(", ".control(", "set_zero_position")
        self.assertFalse(any(token in source for token in forbidden))


if __name__ == "__main__":
    unittest.main()
