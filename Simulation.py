import serial
import time
import math
import os
import opensim as osim

# ================= Setup =================
SERIAL_PORT = 'COM7'  # ⚠️ Port conformation
BAUD_RATE = 115200
MODEL_FILE = r"D:\OpenSim B4\gait2392_simbody.osim"  # File conformation

TARGET_HEIGHT = 0.0

ALPHA = 0.993
DT = 0.02
EMG_SMOOTH = 0.1


# ================= Tool =================
class EMGProcessor:
    def __init__(self):
        self.val = 0.0

    def process(self, raw_val):
        # Normalization
        norm = abs(raw_val) / 4095.0
        if norm > 1.0: norm = 1.0
        self.val = (self.val * (1 - EMG_SMOOTH)) + (norm * EMG_SMOOTH)
        return self.val


# --- Pitch Calculation ---
def calculate_pitch(acc_y, acc_z, gyro_x, current_angle):
    acc_angle = math.degrees(math.atan2(acc_z, acc_y))
    return ALPHA * (current_angle + gyro_x * DT) + (1 - ALPHA) * acc_angle


# --- Roll Calculation ---
def calculate_roll(acc_y, acc_x, gyro_z, current_angle):
    # Rotate around z-axis
    acc_angle = math.degrees(math.atan2(acc_x, acc_y))
    return ALPHA * (current_angle - gyro_z * DT) + (1 - ALPHA) * acc_angle


# ================= Main program =================
def main():
    print(f"Loading the model: {MODEL_FILE}")

    try:
        model = osim.Model(MODEL_FILE)

        # --- Model Initialization ---
        coord_set = model.updCoordinateSet()

        # Set up position
        coord_set.get("pelvis_ty").setDefaultValue(TARGET_HEIGHT)
        coord_set.get("pelvis_tx").setDefaultValue(0)
        coord_set.get("pelvis_tz").setDefaultValue(0)

        # Add Geometry Path
        model_dir = os.path.dirname(MODEL_FILE)
        geometry_path = os.path.join(model_dir, "Geometry")
        osim.ModelVisualizer.addDirToGeometrySearchPaths(geometry_path)

        model.setUseVisualizer(True)
        state = model.initSystem()
        print("✅ System initialises successfully！")

    except Exception as e:
        print(f"❌ System initialises failed: {e}")
        return

    # Setup background
    if model.hasVisualizer():
        try:
            viz = model.getVisualizer().getSimbodyVisualizer()
            viz.setBackgroundType(viz.SolidColor)
            viz.setBackgroundColor(osim.Vec3(0.1, 0.1, 0.1))
        except:
            pass

    print("The muscles and joints are being bound...")

    # 1. Main control
    coord_height = model.updCoordinateSet().get("pelvis_ty")

    # 2. Pitch
    coord_waist_pitch = model.updCoordinateSet().get("lumbar_extension")
    coord_hip_pitch = model.updCoordinateSet().get("hip_flexion_r")

    # 3. Roll/Sway
    coord_waist_roll = model.updCoordinateSet().get("lumbar_bending")  # Lumbar scoliosis
    coord_hip_roll = model.updCoordinateSet().get("hip_adduction_r")  # Hip joint adduction/abduction

    # 4. muscle
    mus_quad = model.updMuscles().get("rect_fem_r")
    try:
        mus_ham = model.updMuscles().get("bifemlh_r")  # Long head of the biceps femoris
    except:
        mus_ham = model.updMuscles().get("semimem_r")

    try:
        mus_waist_r = model.updMuscles().get("ercspn_r")
    except:
        mus_waist_r = None

    # --- Connecting Hardware ---
    print(f"Connecting Hardware {SERIAL_PORT} ...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("✅ Hardware connected!")
    except:
        print("❌ Unable to connect to the serial port. Entering [Demo Mode]")
        ser = None

    # --- Loop ---
    print(f"\n>>> Bidirectional simulation operation (front-back + left-right) <<<")

    leg_pitch = 0.0
    leg_roll = 0.0
    waist_pitch = 0.0
    waist_roll = 0.0

    # 3 processors
    proc_waist = EMGProcessor()
    proc_quad = EMGProcessor()
    proc_ham = EMGProcessor()

    start_time = time.time()

    while True:
        try:
            act_waist = 0.0
            act_quad = 0.0
            act_ham = 0.0

            # Data collection
            if ser is None:  # demonstration
                t = time.time() - start_time
                leg_pitch = 30 * math.sin(t * 2)
                leg_roll = 15 * math.cos(t * 2)  # demonstration
                act_quad = (math.sin(t * 2) + 1) / 2
                time.sleep(0.02)

            elif ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line: continue
                data = line.split(',')

                # Arduino transmission: [Abdominal EMG, Front leg EMG, Back leg EMG, Lumbar MPU(6), Leg MPU(6)]
                # 总A total of 3 + 6 + 6 = 15 pieces of data
                if len(data) >= 15:
                    # --- 1. Analysis EMG ---
                    raw_waist = int(data[0])
                    raw_quad = int(data[1])
                    raw_ham = int(data[2])

                    act_waist = proc_waist.process(raw_waist)
                    act_quad = proc_quad.process(raw_quad)
                    act_ham = proc_ham.process(raw_ham)

                    # --- 2. Analysis: Waist MPU (Arduino's initial model for waist) ---
                    # Search 3-8: AX, AY, AZ, GX, GY, GZ
                    w_ax = float(data[3])  # [For left and right]
                    w_ay = float(data[4])
                    w_az = float(data[5])
                    w_gx = float(data[6])
                    w_gz = float(data[8])  # [For left and right]

                    # --- 3. Analysis: Leg MPU (Arduino-based leg) ---
                    # Search 9-14: AX, AY, AZ, GX, GY, GZ
                    l_ax = float(data[9])  # [For left and right]
                    l_ay = float(data[10])
                    l_az = float(data[11])
                    l_gx = float(data[12])
                    l_gz = float(data[14])  # [For left and right]

                    # --- Core calculation ---
                    # 1. Pitch -  Y, Z, GX
                    waist_pitch = calculate_pitch(w_ay, w_az, w_gx, waist_pitch)
                    leg_pitch = calculate_pitch(l_ay, l_az, l_gx, leg_pitch)

                    # 2. Roll -  Y, X, GZ
                    waist_roll = calculate_roll(w_ay, w_ax, w_gz, waist_roll)
                    leg_roll = calculate_roll(l_ay, l_ax, l_gz, leg_roll)

            # --- Drive the model ---
            coord_height.setValue(state, TARGET_HEIGHT)

            # 2. Pitch
            coord_hip_pitch.setValue(state, math.radians(leg_pitch))
            coord_waist_pitch.setValue(state, math.radians(waist_pitch))

            # 3. Roll
            coord_hip_roll.setValue(state, math.radians(leg_roll))
            coord_waist_roll.setValue(state, math.radians(waist_roll))

            # 4. Drive the muscles
            mus_quad.setActivation(state, act_quad)
            if mus_ham: mus_ham.setActivation(state, act_ham)
            if mus_waist_r: mus_waist_r.setActivation(state, act_waist)

            # --- Update ---
            model.realizePosition(state)
            if model.hasVisualizer():
                model.getVisualizer().getSimbodyVisualizer().drawFrameNow(state)

            # Print log
            print(f"Waist: {waist_pitch:.0f}/{waist_roll:.0f}° | Leg: {leg_pitch:.0f}/{leg_roll:.0f}°", end='\r')

        except KeyboardInterrupt:
            break
        except Exception:
            continue


if __name__ == "__main__":
    main()