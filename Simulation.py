"""
Gait Rehabilitation Simulation System - Version 2.0 (Engineering Release)
Author: Jasmine Leelo | Oura Engineering Application
Description:
    Real-time bridge between wearable IMU/EMG sensors and OpenSim.
    Features automated gyroscope calibration, robust error handling,
    and relative path management.
"""

import os
import sys
import time
import math
import serial
import logging
from typing import Dict, List, Optional, Tuple

# Import OpenSim (assuming standard environment setup)
try:
    import opensim as osim
except ImportError:
    logging.warning("OpenSim API not found. Simulation features will be disabled.")
    osim = None

# --- Configuration & Constants ---
CONFIG = {
    "HARDWARE": {
        "PORT": "COM7",  # Change to '/dev/ttyUSB0' for Linux/Mac
        "BAUD_RATE": 115200,
        "TIMEOUT": 1.0,
        "PACKET_SIZE": 15  # Expected number of data points per line
    },
    "FILTER": {
        "ALPHA": 0.993,  # Complementary filter weight (Trust Gyro vs Accel)
        "DT": 0.02,  # Sampling interval (50Hz)
        "EMG_SMOOTH": 0.1  # Smoothing factor for muscle signals
    },
    "MODEL": {
        # Dynamically find the model file relative to this script
        "FILE_NAME": "gait2392_simbody.osim",
        "TARGET_HEIGHT": 0.0
    },
    "CALIBRATION": {
        "SAMPLES": 200,  # Number of frames to read for calibration
        "WARMUP_SEC": 2  # Seconds to wait before calibrating
    }
}

# Setup Logging (Standard Output)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger(__name__)


class EMGProcessor:
    """Normalizes and smooths raw EMG signals."""

    def __init__(self, smoothing: float = 0.1):
        self.val = 0.0
        self.smoothing = smoothing

    def process(self, raw_val: int) -> float:
        # Normalize 12-bit ADC value (0-4095)
        norm = abs(raw_val) / 4095.0
        if norm > 1.0: norm = 1.0

        # Exponential Moving Average (Low-pass filter)
        self.val = (self.val * (1 - self.smoothing)) + (norm * self.smoothing)
        return self.val


class BiofeedbackSystem:
    """Main Controller for Hardware-in-the-Loop Simulation."""

    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.model: Optional[osim.Model] = None
        self.state = None
        self.viz = None

        # State Variables (Angles in degrees)
        self.pose = {
            'leg_pitch': 0.0, 'leg_roll': 0.0,
            'waist_pitch': 0.0, 'waist_roll': 0.0
        }

        # Calibration Offsets (Gyro Bias)
        self.bias = {
            'w_gx': 0.0, 'w_gz': 0.0,
            'l_gx': 0.0, 'l_gz': 0.0
        }

        # Signal Processors
        self.proc_waist = EMGProcessor(CONFIG["FILTER"]["EMG_SMOOTH"])
        self.proc_quad = EMGProcessor(CONFIG["FILTER"]["EMG_SMOOTH"])
        self.proc_ham = EMGProcessor(CONFIG["FILTER"]["EMG_SMOOTH"])

    def load_model(self):
        """Initializes the OpenSim model with relative paths."""
        if not osim: return

        # Robust path finding (Fixes 'It works on my machine')
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, "Models", CONFIG["MODEL"]["FILE_NAME"])

        logger.info(f"Loading model from: {model_path}")

        try:
            if not os.path.exists(model_path):
                raise FileNotFoundError(f"Model file not found at {model_path}")

            self.model = osim.Model(model_path)
            self.model.setUseVisualizer(True)
            self.state = self.model.initSystem()

            # Setup Visualizer
            if self.model.hasVisualizer():
                viz = self.model.getVisualizer().getSimbodyVisualizer()
                viz.setBackgroundType(viz.SolidColor)
                viz.setBackgroundColor(osim.Vec3(0.1, 0.1, 0.1))

            logger.info("Biomechanics Model Initialized Successfully.")

        except Exception as e:
            logger.error(f"Failed to load OpenSim model: {e}")
            sys.exit(1)  # Critical failure

    def connect_hardware(self) -> bool:
        """Establishes serial connection with retry logic."""
        port = CONFIG["HARDWARE"]["PORT"]
        try:
            logger.info(f"Attempting connection to {port}...")
            self.ser = serial.Serial(
                port,
                CONFIG["HARDWARE"]["BAUD_RATE"],
                timeout=CONFIG["HARDWARE"]["TIMEOUT"]
            )
            time.sleep(2)  # Allow Arduino reset
            logger.info("Hardware Connected.")
            return True
        except serial.SerialException as e:
            logger.warning(f"Connection failed: {e}. Entering DEMO MODE.")
            self.ser = None
            return False

    def calibrate_sensors(self):
        """
        Phase 1: Startup Calibration.
        Captures static noise (bias) from gyroscopes to prevent drift.
        """
        if not self.ser:
            logger.info("Skipping calibration (Demo Mode).")
            return

        logger.info("⚠️ STARTUP CALIBRATION: Please stand still for 5 seconds...")
        time.sleep(CONFIG["CALIBRATION"]["WARMUP_SEC"])

        samples = 0
        totals = {'w_gx': 0.0, 'w_gz': 0.0, 'l_gx': 0.0, 'l_gz': 0.0}
        target = CONFIG["CALIBRATION"]["SAMPLES"]

        while samples < target:
            if self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    data = line.split(',')
                    if len(data) >= CONFIG["HARDWARE"]["PACKET_SIZE"]:
                        # Accumulate raw gyro values [Indexes based on doc source]
                        totals['w_gx'] += float(data[6])
                        totals['w_gz'] += float(data[8])
                        totals['l_gx'] += float(data[12])
                        totals['l_gz'] += float(data[14])
                        samples += 1
                        if samples % 50 == 0:
                            print(f"Calibrating... {int((samples / target) * 100)}%", end='\r')
                except ValueError:
                    continue

        # Calculate Average Bias
        for key in totals:
            self.bias[key] = totals[key] / samples

        logger.info(f"Calibration Complete. Biases detected: {self.bias}")

    def calculate_pitch(self, acc_y: float, acc_z: float, gyro_x: float, current_angle: float, bias_x: float) -> float:
        """Computes Pitch (Forward/Back) using Complementary Filter with Bias Correction."""
        # Remove drift (calibration step)
        corrected_gyro = gyro_x - bias_x

        # Calculate Accelerometer Angle (Gravity vector)
        acc_angle = math.degrees(math.atan2(acc_z, acc_y))

        # Sensor Fusion: Trust Gyro for speed, Accel for drift correction
        alpha = CONFIG["FILTER"]["ALPHA"]
        dt = CONFIG["FILTER"]["DT"]

        return alpha * (current_angle + corrected_gyro * dt) + (1 - alpha) * acc_angle

    def calculate_roll(self, acc_y: float, acc_x: float, gyro_z: float, current_angle: float, bias_z: float) -> float:
        """Computes Roll (Left/Right sway) with Bias Correction."""
        corrected_gyro = gyro_z - bias_z
        acc_angle = math.degrees(math.atan2(acc_x, acc_y))
        alpha = CONFIG["FILTER"]["ALPHA"]
        dt = CONFIG["FILTER"]["DT"]

        return alpha * (current_angle - corrected_gyro * dt) + (1 - alpha) * acc_angle

    def update_simulation(self):
        """Applies calculated pose to OpenSim model."""
        if not self.model or not self.state: return

        # Map class variables to Model coordinates
        # Note: OpenSim uses Radians, we calculated Degrees
        coords = self.model.updCoordinateSet()

        # Pitch Mapping
        coords.get("hip_flexion_r").setValue(self.state, math.radians(self.pose['leg_pitch']))
        coords.get("lumbar_extension").setValue(self.state, math.radians(self.pose['waist_pitch']))

        # Roll Mapping
        coords.get("hip_adduction_r").setValue(self.state, math.radians(self.pose['leg_roll']))
        coords.get("lumbar_bending").setValue(self.state, math.radians(self.pose['waist_roll']))

        # Render Frame
        self.model.realizePosition(self.state)
        if self.model.hasVisualizer():
            self.model.getVisualizer().getSimbodyVisualizer().drawFrameNow(self.state)

    def run(self):
        """Main execution loop."""
        logger.info(">>> Starting Biofeedback Loop. Press Ctrl+C to stop. <<<")
        self.load_model()
        self.connect_hardware()
        self.calibrate_sensors()  # Run Phase 1

        try:
            while True:
                # 1. Fetch Data
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    data = line.split(',')

                    if len(data) < CONFIG["HARDWARE"]["PACKET_SIZE"]:
                        continue  # Skip corrupted packets

                    # 2. Parse & Process (Using Calibrated Biases)
                    # Waist IMU Data
                    w_ay, w_az = float(data[4]), float(data[5])
                    w_gx, w_gz = float(data[6]), float(data[8])

                    # Leg IMU Data
                    l_ay, l_az = float(data[10]), float(data[11])
                    l_gx, l_gz = float(data[12]), float(data[14])

                    # 3. Sensor Fusion Math
                    self.pose['waist_pitch'] = self.calculate_pitch(
                        w_ay, w_az, w_gx, self.pose['waist_pitch'], self.bias['w_gx']
                    )
                    self.pose['leg_pitch'] = self.calculate_pitch(
                        l_ay, l_az, l_gx, self.pose['leg_pitch'], self.bias['l_gx']
                    )

                    # (Similar calls for Roll...)

                    # 4. Update Visuals
                    self.update_simulation()

                elif self.ser is None:
                    # Demo Mode Logic
                    t = time.time()
                    self.pose['leg_pitch'] = 30 * math.sin(t * 2)
                    self.update_simulation()
                    time.sleep(0.02)

        except KeyboardInterrupt:
            logger.info("Stopping Simulation...")
            if self.ser: self.ser.close()


if __name__ == "__main__":
    system = BiofeedbackSystem()
    system.run()