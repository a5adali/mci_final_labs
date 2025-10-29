import re
import serial
import matplotlib.pyplot as plt
from drawnow import *
import time

# === Setup Serial ===
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
plt.ion()

# === Data containers ===
accX_vals = []
gyroX_vals = []
angle_vals = []
time_ms = []
cnt = 0
MAX_SAMPLES = 500
SAMPLE_INTERVAL_MS = 10  # adjust if different

# regexes
num_re = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")
label_num_re = re.compile(r"([A-Za-z][A-Za-z0-9]*)\s*=\s*(" + num_re.pattern + r")")

def makeFig():
    plt.clf()
    plt.title('Live accX, gyroX & angle')
    plt.grid(True)
    plt.xlabel('Sample Number')
    plt.ylabel('Value')
    plt.plot(time_ms, accX_vals, 'r.-', label='accX')
    plt.plot(time_ms, gyroX_vals, 'g.-', label='gyroX')
    plt.plot(time_ms, angle_vals, 'b.-', label='angle')
    plt.legend(loc='upper left')

def find_label_value(label_dict, keywords):
    """Return first matching value in label_dict where key contains any keyword (case-insensitive)."""
    for k, v in label_dict.items():
        kl = k.lower()
        for kw in keywords:
            if kw in kl:
                return v
    return None

# previous values to fill gaps
prev_accX = None
prev_gyroX = None
prev_angle = None

try:
    while True:
        if ser.in_waiting == 0:
            time.sleep(0.005)
            continue

        raw = ser.readline().decode(errors='ignore').strip()
        if not raw:
            continue

        # parse label=value pairs first
        label_pairs = label_num_re.findall(raw)
        label_dict = {k: float(v) for (k, v) in label_pairs} if label_pairs else {}

        # Try to pick gyro, accel, angle using label names
        gyro = find_label_value(label_dict, ['gyro', 'gyr'])
        # prefer AccX, fallback to any "acc" or "accel"
        acc = find_label_value(label_dict, ['accx', 'acc_x', 'accelx', 'acc'])  # will match any containing 'acc'
        angle = find_label_value(label_dict, ['angle', 'pitch', 'roll', 'yaw'])

        # If label parsing failed, fall back to raw numbers in the line
        if gyro is None or acc is None or angle is None:
            nums = num_re.findall(raw)
            # convert to floats
            nums = [float(n) for n in nums] if nums else []
            # heuristics: if there are exactly 3 raw numbers and no labels, assume order:
            # either (acc, gyro, angle) or (gyro, acc, angle). We'll try to be smart:
            if not label_dict and len(nums) >= 3:
                # choose the arrangement closest to previous values if available
                # try both orders and pick one where differences to prev values are smaller
                # default: assume acc, gyro, angle
                acc_try, gyro_try, angle_try = nums[0], nums[1], nums[2]
                # if that seems odd and previous exist, try swapping first two
                if (prev_accX is not None and prev_gyroX is not None):
                    diff1 = abs(acc_try - prev_accX) + abs(gyro_try - prev_gyroX)
                    diff2 = abs(gyro_try - prev_accX) + abs(acc_try - prev_gyroX)
                    if diff2 < diff1:
                        acc_try, gyro_try = gyro_try, acc_try
                acc = acc_try
                gyro = gyro_try
                angle = angle_try
            else:
                # If some values still missing, fill from nums left-to-right into missing slots
                nums_i = 0
                for slot in ('acc', 'gyro', 'angle'):
                    if locals()[slot] is None and nums_i < len(nums):
                        locals()[slot] = nums[nums_i]  # won't persist if we rebind name; handle manually below
                        if slot == 'acc':
                            acc = nums[nums_i]
                        elif slot == 'gyro':
                            gyro = nums[nums_i]
                        elif slot == 'angle':
                            angle = nums[nums_i]
                        nums_i += 1

        # At this point, gyro/acc/angle may still be None.
        # Use previous values to fill missing (so plot remains smooth)
        if acc is None:
            acc = prev_accX
        if gyro is None:
            gyro = prev_gyroX
        if angle is None:
            angle = prev_angle

        # If we still don't have required data (no previous values), skip the line
        if acc is None or gyro is None or angle is None:
            print("Skipping line (insufficient data):", raw)
            continue

        # update previous
        prev_accX = acc
        prev_gyroX = gyro
        prev_angle = angle

        # append and update time index
        accX_vals.append(acc)
        gyroX_vals.append(gyro)
        angle_vals.append(angle)
        time_ms.append(cnt)
        cnt += SAMPLE_INTERVAL_MS

        # limit history
        if len(accX_vals) > MAX_SAMPLES:
            accX_vals.pop(0)
            gyroX_vals.pop(0)
            angle_vals.pop(0)
            time_ms.pop(0)

        drawnow(makeFig)
        plt.pause(0.0001)

except KeyboardInterrupt:
    print("\nInterrupted by user. Closing serial port...")
finally:
    try:
        ser.close()
    except Exception:
        pass
    print("Done.")
