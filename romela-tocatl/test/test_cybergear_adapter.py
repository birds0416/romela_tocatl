from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).parents[1] / "src" / "tocatl_control"
sys.path.insert(0, str(PACKAGE_ROOT))

from tocatl_control.cybergear_adapter import DriverBackedCyberGearAdapter  # noqa: E402


class FakeDriverMotor:
    def __init__(self):
        self.motor_id = 21
        self.state = {
            "position": 0.25,
            "velocity": 1.5,
            "torque": -0.2,
            "temperature": 41.0,
            "bus_voltage": 24.1,
            "mode": "standby",
        }
        self.faults = {"over_temperature": False, "encoder": True}
        self.calls = []

    def request_motor_state(self):
        self.calls.append("request_motor_state")

    def request_motor_fault_status(self):
        self.calls.append("request_motor_fault_status")


class CyberGearAdapterTests(unittest.TestCase):
    def test_read_only_update_calls_only_state_and_fault_requests(self):
        motor = FakeDriverMotor()
        adapter = DriverBackedCyberGearAdapter(motor)
        adapter.request_read_only_update()
        self.assertEqual(motor.calls, ["request_motor_state", "request_motor_fault_status"])

    def test_snapshot_normalizes_driver_state(self):
        adapter = DriverBackedCyberGearAdapter(FakeDriverMotor())
        telemetry = adapter.snapshot(timestamp_s=55.0)
        self.assertEqual(telemetry.timestamp_s, 55.0)
        self.assertEqual(telemetry.motor_id, 21)
        self.assertTrue(telemetry.fresh)
        self.assertEqual(telemetry.position_rad, 0.25)
        self.assertEqual(telemetry.velocity_rad_s, 1.5)
        self.assertEqual(telemetry.torque_estimate_nm, -0.2)
        self.assertEqual(telemetry.temperature_c, 41.0)
        self.assertEqual(telemetry.bus_voltage_v, 24.1)
        self.assertEqual(telemetry.mode, "standby")
        self.assertEqual(telemetry.faults, ("encoder",))

    def test_module_has_no_motion_or_calibration_commands(self):
        source = (PACKAGE_ROOT / "tocatl_control" / "cybergear_adapter.py").read_text()
        forbidden = (".enable(", ".control(", "set_zero_position", ".mode(", ".set_parameter(")
        self.assertFalse(any(token in source for token in forbidden))


if __name__ == "__main__":
    unittest.main()
