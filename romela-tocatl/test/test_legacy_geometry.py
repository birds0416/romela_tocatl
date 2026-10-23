from math import pi
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).parents[1] / "src" / "tocatl_kinematics"
sys.path.insert(0, str(PACKAGE_ROOT))

from tocatl_kinematics.legacy_geometry import (  # noqa: E402
    BODY_REFERENCE,
    DEFAULT_LEG_GEOMETRY,
    Point3,
    apply_legacy_motor_windows,
    foot_position_from_joint_angles,
    inverse_kinematics_from_foot_position,
    legacy_motor_to_theta,
    theta_to_legacy_motor,
)


class LegacyGeometryTests(unittest.TestCase):
    def test_geometry_matches_recovered_controller_and_measurements(self):
        self.assertAlmostEqual(DEFAULT_LEG_GEOMETRY.upper_link_mm, 330.3455)
        self.assertAlmostEqual(DEFAULT_LEG_GEOMETRY.lower_link_mm, 458.0)
        self.assertAlmostEqual(DEFAULT_LEG_GEOMETRY.upper_link_mm / 10.0, 33.03455)
        self.assertAlmostEqual(DEFAULT_LEG_GEOMETRY.lower_link_mm / 10.0, 45.8)

    def test_body_has_six_even_mount_angles_starting_at_30_degrees(self):
        angles = BODY_REFERENCE.leg_mount_angles_rad
        self.assertEqual(len(angles), 6)
        self.assertAlmostEqual(angles[0], pi / 6)
        self.assertTrue(all(abs((angles[i + 1] - angles[i]) - pi / 3) < 1e-12 for i in range(5)))

    def test_legacy_fk_ik_round_trip_on_reachable_pose(self):
        original = (0.25, -0.85, 1.35)
        foot = foot_position_from_joint_angles(*original)
        recovered = inverse_kinematics_from_foot_position(foot)
        for actual, expected in zip(recovered, original):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_theta_motor_transform_round_trip(self):
        theta = (0.2, -0.8, 1.1)
        motor = theta_to_legacy_motor(*theta)
        recovered = legacy_motor_to_theta(*motor)
        for actual, expected in zip(recovered, theta):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_legacy_motor_windows_are_reference_only_and_bound_outputs(self):
        limited = apply_legacy_motor_windows(3.0, -10.0, 20.0)
        motor = theta_to_legacy_motor(*limited)
        self.assertGreaterEqual(motor[0], -1.0)
        self.assertLessEqual(motor[0], 1.0)
        self.assertGreaterEqual(motor[1], 0.0)
        self.assertLessEqual(motor[1], 4.9)
        self.assertGreaterEqual(motor[2] - motor[1], -5.0)
        self.assertLessEqual(motor[2] - motor[1], 0.0)

    def test_unreachable_far_point_clamps_like_legacy_ik_branch(self):
        angles = inverse_kinematics_from_foot_position(Point3(10_000.0, 0.0, 0.0))
        self.assertAlmostEqual(angles[2], 0.0)

    def test_module_has_no_hardware_dependency_or_motion_api(self):
        source = (PACKAGE_ROOT / "tocatl_kinematics" / "legacy_geometry.py").read_text()
        forbidden = ("import can", ".send(", ".enable(", ".control(", "set_zero_position")
        self.assertFalse(any(token in source for token in forbidden))


if __name__ == "__main__":
    unittest.main()
