
import cv2
import mediapipe as mp
import math
import serial
import time
import threading
import numpy as np

# SERIAL SETUP
try:
    ser = serial.Serial('/dev/tty.usbserial-110', 250000, timeout=0)
    time.sleep(2)
except serial.SerialException as e:
    print("Serial error:", e)
    exit()

# MEDIAPIPE SETUP (optimized for faster performance)
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1,
                       min_detection_confidence=0.5, min_tracking_confidence=0.5)

# CAMERA SETUP (lower resolution for speed)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 240)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 180)
cap.set(cv2.CAP_PROP_FPS, 60)
if hasattr(cv2, 'CAP_PROP_BUFFERSIZE'):
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

frame_lock = threading.Lock()
shared_frame = None
running = True
last_angles = None  # For optional filtering

def calc_angle(a, b, c):
    ang = math.degrees(
        math.atan2(c[1] - b[1], c[0] - b[0]) -
        math.atan2(a[1] - b[1], a[0] - b[0])
    )
    return ang + 360 if ang < 0 else ang

def map_ang(x, in_min, in_max, out_min, out_max):
    x = max(min(x, in_max), in_min)
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def get_ang(lm, a, b, c):
    return calc_angle((lm[a].x, lm[a].y), (lm[b].x, lm[b].y), (lm[c].x, lm[c].y))

def camera_thread():
    global shared_frame, running
    while running:
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                shared_frame = frame

def processing_thread():
    global shared_frame, running, last_angles
    prev_time = time.time()

    while running:
        frame = None
        with frame_lock:
            if shared_frame is not None:
                frame = shared_frame

        if frame is None:
            continue

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        if results.multi_hand_landmarks and results.multi_handedness:
            lm = results.multi_hand_landmarks[0].landmark
            hand_label = results.multi_handedness[0].classification[0].label

            if hand_label == "Right":
                # Thumb Angles
                thumb_tip = map_ang(get_ang(lm, 3, 2, 1), 30, 160, 0, 180)
                thumb_mid = map_ang(get_ang(lm, 4, 3, 2), 30, 160, 0, 180)
                thumb_spread_raw = math.degrees(math.atan2(lm[1].y - lm[0].y, lm[1].x - lm[0].x))
                thumb_spread = map_ang(thumb_spread_raw, -30, 60, 0, 180)

                # Other Fingers
                pinky_tip = map_ang(get_ang(lm, 17, 18, 20), 30, 160, 0, 180)
                pinky_base = map_ang(get_ang(lm, 0, 17, 18), 30, 160, 0, 180)

                ring_tip = map_ang(get_ang(lm, 13, 14, 16), 30, 160, 0, 180)
                ring_base = map_ang(get_ang(lm, 0, 13, 14), 30, 160, 0, 180)

                middle_tip = map_ang(get_ang(lm, 9, 10, 12), 30, 160, 0, 180)
                middle_base = map_ang(get_ang(lm, 0, 9, 10), 30, 160, 0, 180)

                index_tip = map_ang(get_ang(lm, 5, 6, 8), 30, 160, 0, 180)
                index_base = map_ang(get_ang(lm, 0, 5, 6), 30, 160, 0, 180)

                angles = np.array([
                    pinky_tip, pinky_base, ring_tip, ring_base,
                    middle_tip, middle_base, index_tip, index_base,
                    thumb_tip, thumb_mid, thumb_spread
                ], dtype=int)

                if not np.array_equal(angles, last_angles):
                    try:
                        ser.write((','.join(map(str, angles)) + '\n').encode())
                        last_angles = angles
                    except:
                        pass

        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time
        print(f"FPS: {fps:.1f}", end='\r')

# START THREADS
t1 = threading.Thread(target=camera_thread)
t2 = threading.Thread(target=processing_thread)

t1.start()
t2.start()

print("Running... Press 'q' to quit.")

while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        running = False
        break

# CLEANUP
t1.join()
t2.join()
cap.release()
ser.close()
cv2.destroyAllWindows()
print("Exited.")

