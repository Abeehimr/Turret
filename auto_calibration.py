import cv2
import serial
import time
import numpy as np

# =========================
# CONFIG
# =========================
FRAME_W, FRAME_H = 640, 480
START_ANGLE = (90, 100)
STEP = 1
ITER_COUNT = 1

MOV_START_TIME = 1.0
MISSING_TIMEOUT = 2.0
MAX_SAMPLES = 15
INF = 260

DIRECTIONS = {
    'right': ( STEP, 0),
    'down':  (0, STEP),
    'left':  (-STEP, 0),
    'up':    (0, -STEP)
}

# =========================
# CAMERA
# =========================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# =========================
# ARDUINO
# =========================
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
arduino.readline()
print("Waiting for Arduino...")
while arduino.readline().decode().strip() != "READY":
    pass
print("Arduino ready!")

def send_to_arduino(pan, tilt):
    arduino.write(f"{int(pan)},{int(tilt)},{0},{0}\n".encode())

# =========================
# VISION
# =========================
def get_laser_center(frame):
    frame = cv2.GaussianBlur(frame, (3, 3), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = (
        cv2.inRange(hsv, (0, 70, 120), (10, 255, 255)) |
        cv2.inRange(hsv, (170, 70, 120), (180, 255, 255))
    )

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, None, 1)
    mask = cv2.dilate(mask, None, 1)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None

    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 150:
        return None

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None

    x, y, w, h = cv2.boundingRect(c)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    return cx, cy, x, y, w, h

def draw_crosshair(frame):
    cx, cy = FRAME_W // 2, FRAME_H // 2
    cv2.drawMarker(frame, (cx, cy), (255, 0, 0),
                   cv2.MARKER_CROSS, 20, 2)

def draw_laser(frame, laser):
    cx, cy, x, y, w, h = laser
    cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
    cv2.circle(frame, (cx, cy), 5, (0,0,255), -1)

# =========================
# HELPERS
# =========================
def wait_motion():
    start = time.time()
    while time.time() - start < MOV_START_TIME:
        pass

# =========================
# EDGE SCAN
# =========================
def scan_edge(direction):
    pan, tilt = START_ANGLE
    send_to_arduino(pan, tilt)
    wait_motion()

    last_seen = time.time()
    last_valid = None

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame = cv2.resize(frame, (FRAME_W, FRAME_H))
        draw_crosshair(frame)

        laser = get_laser_center(frame)
        now = time.time()

        if laser:
            last_seen = now
            last_valid = (pan, tilt)
            draw_laser(frame, laser)

            dp, dt = DIRECTIONS[direction]
            pan += dp
            tilt += dt
            send_to_arduino(pan, tilt)

        if now - last_seen > MISSING_TIMEOUT:
            break

        cv2.imshow("Edge Calibration", frame)
        if cv2.waitKey(1) == 27:
            break

    return last_valid

# =========================
# EDGE CALIBRATION
# =========================
limits = {
    'right': [INF]*ITER_COUNT,
    'down':  [-INF]*ITER_COUNT,
    'left':  [-INF]*ITER_COUNT,
    'up':    [INF]*ITER_COUNT
}

for i in range(ITER_COUNT):
    print(f"\nIteration {i+1}/{ITER_COUNT}")
    for d in ['right', 'down', 'left', 'up']:
        result = scan_edge(d)
        if not result:
            continue

        pan, tilt = result
        if d == 'right': limits[d][i] = min(limits[d][i], pan)
        if d == 'left':  limits[d][i] = max(limits[d][i], pan)
        if d == 'up':    limits[d][i] = min(limits[d][i], tilt)
        if d == 'down':  limits[d][i] = max(limits[d][i], tilt)

# =========================
# BOUNDARIES
# =========================
pan_min  = int(np.mean(limits['right']))
pan_max  = int(np.mean(limits['left']))
tilt_min = int(np.mean(limits['up']))
tilt_max = int(np.mean(limits['down']))

center_pan  = (pan_min + pan_max) // 2
center_tilt = (tilt_min + tilt_max) // 2

print("\n--- Edge Calibration Results ---")
print(f"Pan  : {pan_min} → {pan_max} (center {center_pan})")
print(f"Tilt : {tilt_min} → {tilt_max} (center {center_tilt})")

# =========================
# CENTER OFFSET
# =========================
print("\nCalibrating center offset...")
send_to_arduino(center_pan, center_tilt)
wait_motion()

samples = []
start = time.time()

while len(samples) < MAX_SAMPLES and time.time() - start < MISSING_TIMEOUT:
    ret, frame = cap.read()
    if not ret:
        continue

    frame = cv2.resize(frame, (FRAME_W, FRAME_H))
    draw_crosshair(frame)

    laser = get_laser_center(frame)
    if laser:
        samples.append((laser[0], laser[1]))
        draw_laser(frame, laser)

    cv2.imshow("Center Offset Calibration", frame)
    cv2.waitKey(1)

if samples:
    avg_x = int(np.mean([p[0] for p in samples]))
    avg_y = int(np.mean([p[1] for p in samples]))

    OFFSET_PAN  = (FRAME_W // 2) - avg_x
    OFFSET_TILT = (FRAME_H // 2) - avg_y

    print(f"OFFSET_PAN  = {OFFSET_PAN}")
    print(f"OFFSET_TILT = {OFFSET_TILT}")
else:
    print("Laser not detected at center")

# =========================
# CLEANUP
# =========================
cap.release()
cv2.destroyAllWindows()
print("\nCalibration complete.")
