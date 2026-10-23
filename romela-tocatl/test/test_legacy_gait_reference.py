from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).parents[1] / "src" / "tocatl_gait"
sys.path.insert(0, str(PACKAGE_ROOT))

from tocatl_gait.legacy_reference import (  # noqa: E402
    GAIT_BY_NAME,
    GAITS,
    LEGACY_WALKING_ENVELOPE,
    TRIPOD,
    WAVE_STAGGERED,
    gait_sequence,
    phase_for_leg,
)


class LegacyGaitReferenceTests(unittest.TestCase):
    def test_recovered_gait_sequence(self):
        self.assertEqual(gait_sequence(), ("wave", "ripple", "tripod"))
        self.assertEqual(set(GAIT_BY_NAME), {"wave", "ripple", "tripod"})

    def test_all_gaits_have_six_leg_offsets_and_valid_duty(self):
        for gait in GAITS:
            self.assertEqual(len(gait.phase_offsets), 6)
            self.assertGreater(gait.duty_factor, 0.0)
            self.assertLess(gait.duty_factor, 1.0)
            self.assertAlmostEqual(gait.cycle_time_s, 1.5)

    def test_wave_gait_has_one_swing_leg_at_global_phase_zero(self):
        phases = [phase_for_leg(WAVE_STAGGERED, leg_index, 0.0) for leg_index in range(6)]
        swing_legs = [leg_index for leg_index, (_, is_stance, _) in enumerate(phases) if not is_stance]
        self.assertEqual(swing_legs, [0])

    def test_tripod_gait_splits_legs_into_alternating_sets(self):
        phases = [phase_for_leg(TRIPOD, leg_index, 0.0) for leg_index in range(6)]
        swing_legs = [leg_index + 1 for leg_index, (_, is_stance, _) in enumerate(phases) if not is_stance]
        self.assertEqual(swing_legs, [2, 4, 6])

    def test_phase_rejects_invalid_leg_index(self):
        with self.assertRaises(ValueError):
            phase_for_leg(TRIPOD, 6, 0.0)

    def test_walking_envelope_matches_legacy_controller_values(self):
        self.assertEqual(LEGACY_WALKING_ENVELOPE.command_stream_hz, 200.0)
        self.assertEqual(LEGACY_WALKING_ENVELOPE.default_stride_length_mm, 175.0)
        self.assertEqual(LEGACY_WALKING_ENVELOPE.default_step_height_mm, 215.0)
        self.assertEqual(LEGACY_WALKING_ENVELOPE.foot_speed_max_mm_s, 400.0)
        self.assertEqual(LEGACY_WALKING_ENVELOPE.hip_pitch_support_torque_max_nm, 9.25)

    def test_module_has_no_hardware_dependency_or_motion_api(self):
        source = (PACKAGE_ROOT / "tocatl_gait" / "legacy_reference.py").read_text()
        forbidden = ("import can", ".send(", ".enable(", ".control(", "set_zero_position")
        self.assertFalse(any(token in source for token in forbidden))


if __name__ == "__main__":
    unittest.main()
