import cv2
import numpy as np
import serial
from functools import reduce
import time

# =========================
# CONFIG
# =========================
FRAME_W, FRAME_H = 640, 360

HFOV = 62.0   # camera horizontal FOV in degrees
VFOV = 48.0 

PAN_MAX, PAN_MIN = 132, 52
TILT_MAX, TILT_MIN = 120,73
PAN_CENTER  = (PAN_MIN + PAN_MAX) / 2
TILT_CENTER = (TILT_MIN + TILT_MAX) / 2

aggressive = False          # master switch
STABLE_PIXELS = 5          # max allowed movement (pixels)
STABLE_FRAMES = 30          # how many frames must be stable


OFFSET_PAN = 6
OFFSET_TILT = 5
COLOR = "yellow"

# =========================
# COLOR RANGES (HSV)
# =========================
COLOR_RANGES = {
    "red":    [((0,70,120),(10,255,255)), ((170,70,120),(180,255,255))],
    "green":  [((40,52,72),(80,255,255))],
    "blue":   [((94,80,2),(126,255,255))],
    "yellow": [((20,100,100),(30,255,255))],
    "orange": [((10,100,20),(25,255,255))],
    "pink":   [((140,100,100),(170,255,255))],
    "purple": [((125,50,50),(150,255,255))],
    "white":  [((0,0,200),(180,25,255))],
    "black":  [((0,0,0),(180,255,30))]
}

# =========================
# CAMERA / ARDUINO
# =========================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# =========================
# HELPERS (reusable with calibration)
# =========================
def draw_crosshair(frame):
    cx, cy = FRAME_W // 2, FRAME_H // 2
    cv2.drawMarker(frame, (cx, cy), (255,0,0),
                   cv2.MARKER_CROSS, 20, 2)

def draw_target(frame, cx, cy, x, y, w, h):
    cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
    cv2.circle(frame, (cx,cy), 5, (0,0,255), -1)
    cv2.putText(frame, f"({cx},{cy})",
                (x,y-10), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255,255,255), 2)

def offset(frame):
    cv2.putText(frame,
                f"OFF_PAN={OFFSET_PAN:.1f}  OFF_TILT={OFFSET_TILT:.1f}",
                (10, FRAME_H - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0,255,255), 2)
def angles(frame):
    cv2.putText(frame,
        f"PAN={pan} TILT={tilt}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6, (0,255,0), 2)

def send_to_arduino(pan, tilt,shoot,agr):
    arduino.write(f"{pan},{tilt},{int(shoot)},{int(agr)}\n".encode())

# =========================
# VISION
# =========================
def get_color_mask(hsv, color):
    masks = [
        cv2.inRange(hsv, np.array(low), np.array(high))
        for (low, high) in COLOR_RANGES[color]
    ]
    return reduce(cv2.bitwise_or, masks)

def get_largest_target(mask, min_area=500):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None

    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None

    x, y, w, h = cv2.boundingRect(c)
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    return cx, cy, x, y, w, h

# =========================
# PAN / TILT MAPPING
# =========================
def calc_pt(x, y):
    x += OFFSET_PAN
    y += OFFSET_TILT
    nx = x / FRAME_W
    ny = y / FRAME_H

    pan  = PAN_MAX + nx * -(PAN_MAX - PAN_MIN) + OFFSET_PAN
    tilt = TILT_MIN + ny *  (TILT_MAX - TILT_MIN) + OFFSET_TILT

    pan  = int(max(PAN_MIN,  min(PAN_MAX,  pan)))
    tilt = int(max(TILT_MIN, min(TILT_MAX, tilt)))

    return pan, tilt

def calc_pt2(cx, cy):
    # pixel error from image center
    dx = cx - FRAME_W / 2
    dy = cy - FRAME_H / 2

    # pixel → angular error (camera space)
    ang_x = (dx / FRAME_W) * HFOV
    ang_y = (dy / FRAME_H) * VFOV

    # angular error → servo target
    pan  = PAN_CENTER  - ang_x
    tilt = TILT_CENTER + ang_y
    pan += OFFSET_PAN
    tilt += OFFSET_TILT

    # clamp to mechanical limits
    pan  = int(max(PAN_MIN,  min(PAN_MAX,  pan)))
    tilt = int(max(TILT_MIN, min(TILT_MAX, tilt)))

    return pan, tilt



# =========================
# Arduino setup
# =========================
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

line = arduino.readline()

print("Waiting for Arduino...")
while arduino.readline().decode().strip() != "READY":
    pass
print("Arduino ready!")

# =========================
# MAIN LOOP
# =========================

stable_count = 0
prev_cx = None
prev_cy = None

shoot = False
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (FRAME_W, FRAME_H))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = get_color_mask(hsv, COLOR)
    target = get_largest_target(mask)

    draw_crosshair(frame)
    offset(frame)
    # angles(frame)

    if target:
        cx, cy, x, y, w, h = target
        draw_target(frame, cx, cy, x, y, w, h)

        pan, tilt = calc_pt2(cx, cy)

        # ---------- STABILITY CHECK ----------
        if aggressive:
            if prev_cx is not None:
                dx = abs(cx - prev_cx)
                dy = abs(cy - prev_cy)

                if dx < STABLE_PIXELS and dy < STABLE_PIXELS:
                    stable_count += 1
                else:
                    stable_count = 0

            prev_cx, prev_cy = cx, cy

            if stable_count >= STABLE_FRAMES:
                shoot = True
                stable_count = 0  # fire once
        # ------------------------------------


        send_to_arduino(pan, tilt,shoot,aggressive)
        if shoot:
            shoot = False

        # print(f"{COLOR} detected @ ({cx},{cy}) → pan={pan}, tilt={tilt}")

    cv2.imshow("Mask Debug", mask)
    cv2.imshow("Object Tracking", frame)

    key = cv2.waitKey(1) & 0xFF

    STEP = 1   # degrees per key press

    if key == ord('a'):
        OFFSET_PAN += STEP
    elif key == ord('d'):
        OFFSET_PAN -= STEP
    elif key == ord('w'):
        OFFSET_TILT -= STEP
    elif key == ord('s'):
        OFFSET_TILT += STEP
    elif key == ord('q'):
        OFFSET_PAN = 0
        OFFSET_TILT = 0
    elif key == ord('f'):   # press F to trigger
        shoot = True
    elif key == ord('z'):   # press F to trigger
        aggressive = not aggressive

    elif key == 27:  # ESC
        break


cap.release()
cv2.destroyAllWindows()
