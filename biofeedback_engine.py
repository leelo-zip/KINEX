"""
Biofeedback Engine & Telemetry Streamer
Author: Jasmine Leelo
Description: Real-time sensor fusion pipeline. Captures IMU/EMG data,
performs heuristic gait analysis, and broadcasts telemetry via UDP.
"""

import socket
import json
import time
import math
import csv
import logging
import serial
from dataclasses import dataclass
from datetime import datetime
from typing import Tuple, Optional, List


# --- Configuration Management ---
@dataclass
class SystemConfig:
    """Centralized configuration for portability and tuning."""
    SERIAL_PORT: str = 'COM7'
    BAUD_RATE: int = 115200
    UDP_IP: str = "127.0.0.1"
    UDP_PORT: int = 5005
    # Signal Processing
    ADC_MAX: float = 4095.0
    ALPHA: float = 0.98  # Complementary filter weight
    DT: float = 0.02  # Loop time estimate (50Hz)
    # Biofeedback Thresholds
    THIGH_THRESHOLD: float = 0.3
    WAIST_THRESHOLD: float = 0.4


CONFIG = SystemConfig()

# --- Logging Setup ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)s | %(message)s'
)
logger = logging.getLogger(__name__)


# --- Utility Classes ---

class SessionLogger:
    """Handles CSV logging using a context manager for efficient File I/O."""

    def __init__(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"Rehab_Data_{timestamp}.csv"
        self.file = None
        self.writer = None

    def __enter__(self):
        """Opens file once when the session starts."""
        self.file = open(self.filename, mode='w', newline='')
        self.writer = csv.writer(self.file)
        header = ["Time", "Leg_Ang", "Waist_Ang", "EMG_Thigh", "EMG_Waist_L", "EMG_Waist_R", "Score", "Diagnosis"]
        self.writer.writerow(header)
        logger.info(f"📝 Recording Session: {self.filename}")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Safely closes file when session ends or crashes."""
        if self.file:
            self.file.close()
            logger.info("📁 Log file closed safely.")

    def log_frame(self, row_data: List[str]):
        if self.writer:
            self.writer.writerow(row_data)


class MovementEvaluator:
    """Heuristic-based logic to evaluate gait quality (formerly AICoach)."""

    def analyze(self, leg_angle: float, emg_thigh: float, emg_waist_l: float, emg_waist_r: float) -> Tuple[float, str]:
        waist_comp = max(emg_waist_l, emg_waist_r)

        # Default State
        diagnosis = "Idle"
        score = 100.0

        # Heuristic Logic
        if leg_angle > 15:
            if emg_thigh > CONFIG.THIGH_THRESHOLD:
                diagnosis = "🟢 Standard"
                # Dynamic scoring based on waist quietness
                score = max(0, 100 - (waist_comp * 20))
            elif emg_thigh < CONFIG.THIGH_THRESHOLD and waist_comp > CONFIG.WAIST_THRESHOLD:
                diagnosis = "🔴 Compensation"
                score = 40.0
            else:
                diagnosis = "🟡 Inertial"
                score = 60.0

        return score, diagnosis


class SignalProcessor:
    """Filters and normalizes raw sensor streams."""

    def __init__(self):
        self.val = 0.0

    def process_emg(self, raw_val: int) -> float:
        # Normalize
        norm = abs(raw_val) / CONFIG.ADC_MAX
        norm = min(norm, 1.0)  # Clamp to 1.0

        # Low-pass filter (Exponential Moving Average)
        self.val = (self.val * 0.9) + (norm * 0.1)
        return self.val

    @staticmethod
    def calculate_pitch(acc_y: float, acc_z: float, gyro_x: float, current_angle: float) -> float:
        """Complementary Filter for Sensor Fusion."""
        acc_angle = math.degrees(math.atan2(acc_z, acc_y))
        # Trust Gyro (fast) + Correct with Accel (drift-free)
        return CONFIG.ALPHA * (current_angle + gyro_x * CONFIG.DT) + (1 - CONFIG.ALPHA) * acc_angle


# --- Main Application ---

def run_biofeedback_loop():
    logger.info(">>> Biofeedback Engine Starting <<<")

    # 1. Setup Networking
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 2. Setup Hardware
    ser = None
    try:
        ser = serial.Serial(CONFIG.SERIAL_PORT, CONFIG.BAUD_RATE, timeout=1)
        logger.info(f"✅ Hardware Connected on {CONFIG.SERIAL_PORT}")
    except serial.SerialException as e:
        logger.warning(f"❌ Hardware failed: {e}. Entering DEMO MODE.")

    # 3. Initialize Components
    evaluator = MovementEvaluator()
    emg_thigh_proc = SignalProcessor()
    emg_waist_l_proc = SignalProcessor()
    emg_waist_r_proc = SignalProcessor()

    # State Variables
    state = {
        "leg_angle": 0.0,
        "waist_angle": 0.0,
        "start_time": time.time()
    }

    # 4. Main Loop with Context Manager
    with SessionLogger() as recorder:
        try:
            while True:
                curr_time = time.time() - state["start_time"]

                # --- Data Acquisition ---
                act_thigh, act_w_l, act_w_r = 0.0, 0.0, 0.0

                # Hardware Path
                if ser and ser.in_waiting:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if not line: continue

                        data = line.split(',')
                        if len(data) >= 15:  # Input Sanitization
                            # Process EMG
                            act_w_l = emg_waist_l_proc.process_emg(int(data[0]))
                            act_w_r = emg_waist_r_proc.process_emg(int(data[1]))
                            act_thigh = emg_thigh_proc.process_emg(int(data[2]))

                            # Process IMU (Sensor Fusion)
                            # [Index mapping based on source: 197-219]
                            state["waist_angle"] = SignalProcessor.calculate_pitch(
                                float(data[4]), float(data[5]), float(data[6]), state["waist_angle"]
                            )
                            state["leg_angle"] = SignalProcessor.calculate_pitch(
                                float(data[10]), float(data[11]), float(data[12]), state["leg_angle"]
                            )
                    except ValueError:
                        continue  # Skip corrupted packets

                # Demo Mode Path
                elif ser is None:
                    state["leg_angle"] = 45 * abs(math.sin(curr_time))
                    act_thigh = abs(math.sin(curr_time))
                    time.sleep(CONFIG.DT)

                # --- Analysis ---
                score, diag = evaluator.analyze(state["leg_angle"], act_thigh, act_w_l, act_w_r)

                # --- Logging ---
                row_data = [
                    f"{curr_time:.2f}",
                    f"{state['leg_angle']:.1f}", f"{state['waist_angle']:.1f}",
                    f"{act_thigh:.3f}", f"{act_w_l:.3f}", f"{act_w_r:.3f}",
                    f"{score:.1f}", diag
                ]
                recorder.log_frame(row_data)

                # --- Telemetry (UDP) ---
                packet = {
                    "leg": state["leg_angle"],
                    "waist": state["waist_angle"],
                    "act_thigh": act_thigh,
                    "diagnosis": diag
                }
                udp_sock.sendto(json.dumps(packet).encode(), (CONFIG.UDP_IP, CONFIG.UDP_PORT))

                # --- Display ---
                print(f"📡 Telemetry: Leg {state['leg_angle']:5.1f}° | Score {score:3.0f} | {diag}      ", end='\r')

        except KeyboardInterrupt:
            logger.info("\n🛑 System stopped by user.")
        finally:
            if ser: ser.close()


if __name__ == "__main__":
    run_biofeedback_loop()