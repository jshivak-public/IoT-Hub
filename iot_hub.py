import os
os.environ["GPIOZERO_PIN_FACTORY"] = "rpigpio"

import time
import threading
from datetime import datetime
import subprocess
import serial

from gpiozero import MotionSensor

# UART config (Nano over GPIO)
SERIAL_PORT = "/dev/serial0"
BAUD = 9600

latest_nano = {}
nano_lock = threading.Lock()

def parse_kv_line(line):
    # Accept both comma and space separated key=value
    out = {}
    s = line.strip().replace(",", " ")
    parts = s.split()
    for p in parts:
        if "=" in p:
            k, v = p.split("=", 1)
            out[k.strip()] = v.strip()
    return out

def nano_reader_thread():
    global latest_nano
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
        time.sleep(1.5)
    except Exception:
        return

    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            data = parse_kv_line(line)
            if data:
                with nano_lock:
                    latest_nano = data
        except Exception:
            time.sleep(1)

# Camera (rpicam-still)
# Capture on PIR edges
PHOTO_DIR = "/home/jimmy/pir_photos"
os.makedirs(PHOTO_DIR, exist_ok=True)

CAM_CMD = "/usr/bin/rpicam-still"
MIN_SECONDS_BETWEEN_PHOTOS = 2.0
_last_photo_time = 0.0
_photo_lock = threading.Lock()

# These are ONLY used for table output, no extra print spam
last_photo_name = "NA"
photo_event_pending = ""  # "", "ON", or "OFF"
photo_lock2 = threading.Lock()

def take_photo(event_name):
    global _last_photo_time, last_photo_name, photo_event_pending

    if not os.path.exists(CAM_CMD):
        return

    with _photo_lock:
        now_t = time.time()
        if now_t - _last_photo_time < MIN_SECONDS_BETWEEN_PHOTOS:
            return
        _last_photo_time = now_t

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(PHOTO_DIR, "pir_" + event_name.lower() + "_" + ts + ".jpg")
    cmd = [CAM_CMD, "-n", "-t", "200", "-o", filename]

    try:
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        with photo_lock2:
            last_photo_name = os.path.basename(filename)
            photo_event_pending = event_name.upper()  # "ON" or "OFF"
    except Exception:
        # do not print anything (keeps terminal clean)
        pass

# PIR
PIR_PIN = 17
pir = MotionSensor(PIR_PIN)

pir_state = {"motion": False}

def pir_motion():
    pir_state["motion"] = True
    threading.Thread(target=take_photo, args=("ON",), daemon=True).start()

def pir_no_motion():
    pir_state["motion"] = False
    threading.Thread(target=take_photo, args=("OFF",), daemon=True).start()

pir.when_motion = pir_motion
pir.when_no_motion = pir_no_motion

# DHT11 (Adafruit CircuitPython)
try:
    import board
    import adafruit_dht
    dht = adafruit_dht.DHT11(board.D4)
except Exception:
    dht = None

def read_dht():
    if dht is None:
        return None, None
    for _ in range(2):
        try:
            return dht.temperature, dht.humidity
        except Exception:
            time.sleep(0.2)
    return None, None

# MPU6050 (I2C via smbus2)
try:
    from smbus2 import SMBus
    bus = SMBus(1)
    MPU_ADDR = 0x68
    MPU_OK = True
except Exception:
    MPU_OK = False

def mpu_write(reg, val):
    bus.write_byte_data(MPU_ADDR, reg, val)

def mpu_read_word(reg):
    hi = bus.read_byte_data(MPU_ADDR, reg)
    lo = bus.read_byte_data(MPU_ADDR, reg + 1)
    value = (hi << 8) | lo
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

def init_mpu():
    mpu_write(0x6B, 0x00)
    time.sleep(0.1)

def read_mpu6050():
    if not MPU_OK:
        return None
    try:
        ax = mpu_read_word(0x3B) / 16384.0
        ay = mpu_read_word(0x3D) / 16384.0
        az = mpu_read_word(0x3F) / 16384.0
        gx = mpu_read_word(0x43) / 131.0
        gy = mpu_read_word(0x45) / 131.0
        gz = mpu_read_word(0x47) / 131.0
        temp_raw = mpu_read_word(0x41)
        temp_c = (temp_raw / 340.0) + 36.53
        return ax, ay, az, gx, gy, gz, temp_c
    except Exception:
        return None

# Fixed-width table formatting
# Each column has an explicit width so values always line up under headers.
COLS = [
    ("TIME", 8),
    ("PIR", 3),
    ("DHT_T", 6),
    ("DHT_H", 6),
    ("AX", 6),
    ("AY", 6),
    ("AZ", 6),
    ("GX", 6),
    ("GY", 6),
    ("GZ", 6),
    ("MPU_T", 6),
    ("MQ", 4),
    ("DIST", 5),
    ("T_NANO", 7),
    ("PRESS", 7),
    ("PHOTO", 6),
]

def pad(s, w):
    s = str(s)
    if len(s) > w:
        s = s[:w]
    return s.rjust(w)

def fmt_float(v, w, d):
    if v is None:
        return pad("NA", w)
    try:
        return pad(("{0:." + str(d) + "f}").format(float(v)), w)
    except Exception:
        return pad("NA", w)

def fmt_intish(v, w):
    if v is None:
        return pad("NA", w)
    return pad(v, w)

def nano_get(nano, key):
    if not nano:
        return None
    return nano.get(key)

def print_header():
    line = ""
    for name, w in COLS:
        line += pad(name, w) + " "
    print(line.rstrip())
    print("-" * len(line.rstrip()))

def pop_photo_event():
    global photo_event_pending
    with photo_lock2:
        pe = photo_event_pending
        photo_event_pending = ""
    return pe

# Main loop
def main():
    if MPU_OK:
        try:
            init_mpu()
        except Exception:
            pass

    threading.Thread(target=nano_reader_thread, daemon=True).start()

    print("Hub demo running. Ctrl+C to stop.")
    print("Photos dir: " + PHOTO_DIR)
    print_header()

    while True:
        now = datetime.now().strftime("%H:%M:%S")
        pir_v = "1" if pir_state["motion"] else "0"

        dht_t, dht_h = read_dht()
        mpu = read_mpu6050()

        if mpu is None:
            ax = ay = az = gx = gy = gz = tc = None
        else:
            ax, ay, az, gx, gy, gz, tc = mpu

        with nano_lock:
            nano = dict(latest_nano)

        mq = nano_get(nano, "MQ")
        dist = nano_get(nano, "DIST_CM")
        tn = nano_get(nano, "TEMP_C")
        press = nano_get(nano, "PRESS_PA")

        # one-time PHOTO event marker, no extra print lines
        pe = pop_photo_event()
        photo_cell = pe if pe else "NA"


        # Build row in the exact same order/width as header
        row = ""
        row += pad(now, 8) + " "
        row += pad(pir_v, 3) + " "
        row += fmt_float(dht_t, 6, 1) + " "
        row += fmt_intish(dht_h, 6) + " "
        row += fmt_float(ax, 6, 2) + " "
        row += fmt_float(ay, 6, 2) + " "
        row += fmt_float(az, 6, 2) + " "
        row += fmt_float(gx, 6, 1) + " "
        row += fmt_float(gy, 6, 1) + " "
        row += fmt_float(gz, 6, 1) + " "
        row += fmt_float(tc, 6, 1) + " "
        row += fmt_intish(mq, 4) + " "
        row += fmt_intish(dist, 5) + " "
        row += fmt_float(tn, 7, 1) + " "
        row += fmt_intish(press, 7) + " "
        row += pad(photo_cell, 6)

        print(row, flush=True)
        time.sleep(2.0)

if __name__ == "__main__":
    main()
