from pathlib import Path
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "tocatl_control"))
sys.path.insert(0, str(REPO_ROOT / "src" / "tocatl_diagnostics"))

from tocatl_diagnostics.cybergear_telemetry import (  # noqa: E402
    CSV_FIELDS,
    dry_run_payload,
    request_read_only_updates,
    snapshot_from_adapter,
    write_snapshots_csv,
)
from tocatl_diagnostics.motor_map import Motor  # noqa: E402
from tocatl_control.cybergear_adapter import DriverBackedCyberGearAdapter  # noqa: E402


class FakeMotor:
    def __init__(self):
        self.motor_id = 11
        self.state = {
            "position": 1.25,
            "velocity": -0.5,
            "torque": 0.1,
            "temperature": 36.0,
            "bus_voltage": 24.8,
            "mode": "standby",
        }
        self.faults = {"over_temperature": False, "under_voltage": True}
        self.calls = []

    def request_motor_state(self):
        self.calls.append("request_motor_state")

    def request_motor_fault_status(self):
        self.calls.append("request_motor_fault_status")


class CyberGearTelemetryTests(unittest.TestCase):
    def test_dry_run_lists_all_motors_and_read_only_requests(self):
        payload = dry_run_payload()
        self.assertEqual(len(payload["motors"]), 18)
        self.assertEqual(payload["read_only_requests"], [
            "request_motor_state",
            "request_motor_fault_status",
        ])
        self.assertEqual(payload["csv_fields"], list(CSV_FIELDS))

    def test_request_read_only_updates_uses_only_read_methods(self):
        motor = FakeMotor()
        request_read_only_updates([DriverBackedCyberGearAdapter(motor)])
        self.assertEqual(motor.calls, ["request_motor_state", "request_motor_fault_status"])

    def test_snapshot_extracts_known_state_fields_and_active_faults(self):
        snapshot = snapshot_from_adapter(
            Motor(motor_id=11, leg=1, joint="hip_yaw", bus="can1"),
            DriverBackedCyberGearAdapter(FakeMotor()),
            timestamp_s=123.0,
        )
        self.assertTrue(snapshot.fresh)
        self.assertEqual(snapshot.position_rad, 1.25)
        self.assertEqual(snapshot.velocity_rad_s, -0.5)
        self.assertEqual(snapshot.torque_estimate_nm, 0.1)
        self.assertEqual(snapshot.temperature_c, 36.0)
        self.assertEqual(snapshot.bus_voltage_v, 24.8)
        self.assertEqual(snapshot.mode, "standby")
        self.assertEqual(snapshot.faults, "under_voltage")

    def test_csv_writer_uses_stable_header(self):
        snapshot = snapshot_from_adapter(
            Motor(motor_id=11, leg=1, joint="hip_yaw", bus="can1"),
            DriverBackedCyberGearAdapter(FakeMotor()),
            timestamp_s=123.0,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "telemetry.csv"
            write_snapshots_csv(path, [snapshot])
            lines = path.read_text().splitlines()
        self.assertEqual(lines[0].split(","), list(CSV_FIELDS))
        self.assertIn("under_voltage", lines[1])

    def test_module_does_not_call_motion_or_calibration_apis(self):
        source = (REPO_ROOT / "src" / "tocatl_diagnostics" / "tocatl_diagnostics" / "cybergear_telemetry.py").read_text()
        forbidden = (".enable(", ".control(", "set_zero_position", ".mode(", ".set_parameter(")
        self.assertFalse(any(token in source for token in forbidden))


if __name__ == "__main__":
    unittest.main()
