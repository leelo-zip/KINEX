import unittest
import sys
import os

# --- 🛠️ PATH FIX: FORCE PYTHON TO LOOK UPSTAIRS ---
# Get the directory where this test file lives
current_test_dir = os.path.dirname(os.path.abspath(__file__))
# Get the parent directory (the KINEX folder)
parent_dir = os.path.dirname(current_test_dir)

# Print it so we can debug if it fails again
print(f"DEBUG: Adding path to system: {parent_dir}")
sys.path.insert(0, parent_dir)

# --- NOW IMPORT THE ENGINE ---
try:
    from biofeedback_engine import MovementEvaluator, SignalProcessor, SystemConfig
except ImportError as e:
    print("\n\n❌ CRITICAL IMPORT ERROR ❌")
    print(f"Python looked in: {sys.path}")
    print("Please check that 'biofeedback_engine.py' exists in the KINEX folder.")
    raise e


class TestBiofeedbackLogic(unittest.TestCase):

    def setUp(self):
        self.evaluator = MovementEvaluator()
        self.processor = SignalProcessor()
        self.config = SystemConfig(THIGH_THRESHOLD=0.3, WAIST_THRESHOLD=0.4)

    def test_standard_movement(self):
        score, diag = self.evaluator.analyze(leg_angle=30.0, emg_thigh=0.8, emg_waist_l=0.1, emg_waist_r=0.1)
        self.assertEqual(diag, "🟢 Standard")
        self.assertGreater(score, 80.0)

    def test_compensation_detection(self):
        score, diag = self.evaluator.analyze(leg_angle=30.0, emg_thigh=0.1, emg_waist_l=0.6, emg_waist_r=0.1)
        self.assertEqual(diag, "🔴 Compensation")
        self.assertEqual(score, 40.0)

    def test_signal_normalization(self):
        """Test that EMG signals are clamped between 0.0 and 1.0."""
        # Input 5000 (above max 4095) -> Should clamp to 1.0
        result = self.processor.process_emg(5000)
        self.assertLessEqual(result, 1.0)

        # FIX: Create a FRESH processor to test the zero case
        # (Otherwise the old '1.0' value will smooth into this one)
        fresh_processor = SignalProcessor()
        result = fresh_processor.process_emg(0)
        self.assertEqual(result, 0.0)


if __name__ == '__main__':
    unittest.main()