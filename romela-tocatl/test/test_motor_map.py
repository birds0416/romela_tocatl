from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).parents[1] / "src" / "tocatl_diagnostics"
sys.path.insert(0, str(PACKAGE_ROOT))

from tocatl_diagnostics.motor_map import CAN_BITRATE, MOTORS  # noqa: E402


class MotorMapTests(unittest.TestCase):
    def test_inventory_has_all_18_unique_motors(self):
        self.assertEqual(len(MOTORS), 18)
        self.assertEqual(len({motor.motor_id for motor in MOTORS}), 18)

    def test_bus_assignment_matches_deployed_controller(self):
        by_leg = {
            leg: {motor.bus for motor in MOTORS if motor.leg == leg}
            for leg in range(1, 7)
        }
        self.assertEqual(by_leg, {
            1: {"can1"}, 2: {"can0"}, 3: {"can0"},
            4: {"can0"}, 5: {"can1"}, 6: {"can1"},
        })
        self.assertEqual(CAN_BITRATE, 1_000_000)

    def test_joint_numbering_is_consistent(self):
        suffix = {"hip_yaw": 1, "hip_pitch": 2, "knee": 3}
        self.assertTrue(all(
            motor.motor_id == motor.leg * 10 + suffix[motor.joint]
            for motor in MOTORS
        ))

    def test_inventory_module_has_no_hardware_dependency_or_command_api(self):
        source = (PACKAGE_ROOT / "tocatl_diagnostics" / "motor_map.py").read_text()
        forbidden = ("import can", ".send(", ".enable(", "set_zero_position", ".control(")
        self.assertFalse(any(token in source for token in forbidden))


if __name__ == "__main__":
    unittest.main()
