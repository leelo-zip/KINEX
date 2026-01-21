import serial
import time
import math
import csv
import datetime
import json
import socket

# ================= Configuration Area =================
SERIAL_PORT = 'COM7'  # ⚠️ Hardware connection port
BAUD_RATE = 115200

# UDP Communication Config (Sender)
UDP_IP = "127.0.0.1"  # Local address
UDP_PORT = 5005       # Port number

# Threshold Settings
THIGH_THRESHOLD = 0.3
WAIST_THRESHOLD = 0.4


# ================= Utility Classes =================
class DataRecorder:
    def __init__(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"Rehab_Data_{timestamp}.csv"
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(
                ["Time", "Leg_Ang", "Waist_Ang", "EMG_Thigh", "EMG_Waist_L", "EMG_Waist_R", "Score", "Diagnosis"])
        print(f"📝 Recording started: {self.filename}")

    def save(self, row_data):
        with open(self.filename, mode='a', newline='') as file:
            csv.writer(file).writerow(row_data)


class AICoach:
    def analyze(self, leg_angle, emg_thigh, emg_waist_l, emg_waist_r):
        waist_comp = max(emg_waist_l, emg_waist_r)
        diagnosis = "Idle"
        score = 100.0

        if leg_angle > 15:
            if emg_thigh > THIGH_THRESHOLD:
                diagnosis = "🟢 Standard"
                score = 100 - (waist_comp * 20)
            elif emg_thigh < THIGH_THRESHOLD and waist_comp > WAIST_THRESHOLD:
                diagnosis = "🔴 Compensation"
                score = 40.0
            else:
                diagnosis = "🟡 Inertial"
                score = 60.0
        return max(0, score), diagnosis


class EMGProcessor:
    def __init__(self):
        self.val = 0.0

    def process(self, raw):
        norm = abs(raw) / 4095.0
        if norm > 1.0: norm = 1.0
        self.val = (self.val * 0.9) + (norm * 0.1)
        return self.val


def calculate_pitch(acc_y, acc_z, gyro_x, current_angle):
    acc_angle = math.degrees(math.atan2(acc_z, acc_y))
    return 0.98 * (current_angle + gyro_x * 0.02) + 0.02 * acc_angle


# ================= Main Program =================
def main():
    print(">>> Core System Starting (Hardware + AI + Recording) <<<")

    # 1. Connect UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 2. Connect Hardware
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"✅ Hardware Connected: {SERIAL_PORT}")
    except:
        print("❌ Hardware not connected, entering [Demo Mode]")
        ser = None

    recorder = DataRecorder()
    coach = AICoach()
    proc_thigh = EMGProcessor()
    proc_w_l = EMGProcessor()
    proc_w_r = EMGProcessor()

    angle_leg = 0.0
    angle_waist = 0.0
    start_time = time.time()

    print("Sending data to OpenSim...")

    while True:
        try:
            curr_time = time.time() - start_time

            # --- A. Data Acquisition ---
            act_thigh, act_w_l, act_w_r = 0.0, 0.0, 0.0

            if ser and ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line: continue
                data = line.split(',')
                if len(data) >= 15:
                    act_w_l = proc_w_l.process(int(data[0]))
                    act_w_r = proc_w_r.process(int(data[1]))
                    act_thigh = proc_thigh.process(int(data[2]))

                    w_ay, w_az, w_gx = float(data[4]), float(data[5]), float(data[6])
                    angle_waist = calculate_pitch(w_ay, w_az, w_gx, angle_waist)

                    l_ay, l_az, l_gx = float(data[10]), float(data[11]), float(data[12])
                    angle_leg = calculate_pitch(l_ay, l_az, l_gx, angle_leg)

            elif ser is None:  # Demo data
                angle_leg = 45 * abs(math.sin(curr_time))
                act_thigh = abs(math.sin(curr_time))
                time.sleep(0.02)

            # --- B. AI Analysis ---
            score, diag = coach.analyze(angle_leg, act_thigh, act_w_l, act_w_r)

            # --- C. Save Data ---
            row_data = [f"{curr_time:.2f}", f"{angle_leg:.1f}", f"{angle_waist:.1f}",
                        f"{act_thigh:.3f}", f"{act_w_l:.3f}", f"{act_w_r:.3f}",
                        f"{score:.1f}", diag]
            recorder.save(row_data)

            # --- D. Send to OpenSim (UDP Packet) ---
            packet = {
                "leg": angle_leg,
                "waist": angle_waist,
                "act_thigh": act_thigh,
                "act_w_r": act_w_r,
                "act_w_l": act_w_l
            }
            # Send JSON string
            sock.sendto(json.dumps(packet).encode(), (UDP_IP, UDP_PORT))

            # --- E. Terminal Display ---
            print(f"Sending... Leg Ang:{angle_leg:5.1f}° | Score:{score:4.1f} | Status: {diag}      ", end='\r')

        except KeyboardInterrupt:
            print("\nSystem Stopped.")
            break
        except Exception:
            continue


if __name__ == "__main__":
    main()