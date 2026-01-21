import unittest
from biofeedback_engine import MovementEvaluator, SignalProcessor, SystemConfig


class TestBiofeedbackLogic(unittest.TestCase):

    def setUp(self):
        """Runs before every test."""
        self.evaluator = MovementEvaluator()
        self.processor = SignalProcessor()
        # Mock config for consistent testing
        self.config = SystemConfig(THIGH_THRESHOLD=0.3, WAIST_THRESHOLD=0.4)

    def test_standard_movement(self):
        """Test that high leg angle + high thigh activation = Standard."""
        # Leg > 15 deg, Thigh > 0.3
        score, diag = self.evaluator.analyze(leg_angle=30.0, emg_thigh=0.8, emg_waist_l=0.1, emg_waist_r=0.1)
        self.assertEqual(diag, "🟢 Standard")
        self.assertGreater(score, 80.0)  # Score should be high

    def test_compensation_detection(self):
        """Test that back usage instead of leg usage triggers Compensation."""
        # Leg > 15 deg, Thigh LOW (<0.3), Waist HIGH (>0.4)
        score, diag = self.evaluator.analyze(leg_angle=30.0, emg_thigh=0.1, emg_waist_l=0.6, emg_waist_r=0.1)
        self.assertEqual(diag, "🔴 Compensation")
        self.assertEqual(score, 40.0)

    def test_signal_normalization(self):
        """Test that EMG signals are clamped between 0.0 and 1.0."""
        # Input 5000 (above max 4095)
        result = self.processor.process_emg(5000)
        self.assertLessEqual(result, 1.0)

        # Input 0
        result = self.processor.process_emg(0)
        self.assertEqual(result, 0.0)


if __name__ == '__main__':
    unittest.main()
