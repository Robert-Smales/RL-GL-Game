"""
Red Light Green Light — game orchestrator.

Green light: idle, camera off, no inference.
Red light:   run pose estimation for RED_LIGHT_DURATION seconds,
             compare START vs END pose,
             find player with most real movement,
             aim turret at their centre.
"""

import os
import sys
import random
import threading
import time
from pathlib import Path

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import pygame
import serial
import hailo
from hailo_apps_infra.hailo_rpi_common import get_caps_from_pad, app_callback_class

sys.path.insert(0, str(Path(__file__).parent / 'pose_estimation'))
from pose_pipeline import GStreamerPoseEstimationApp

# Config 
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

GREEN_MIN_S = 10
GREEN_MAX_S = 15
RED_MIN_S = 10
RED_MAX_S = 15

TRACK_DURATION_S = 2.5
LASER_DURATION_S = 1.0
RETURN_SETTLE_S = 1.5

STEPS_PER_PIXEL = 3.0
MAX_PAN_STEPS = 5000
CENTRE_TOL_PX = 15
TRACK_CMD_INTERVAL = 0.15

# Movement tuning
STABLE_KEYPOINTS = [5, 6, 11, 12]  # shoulders + hips
PER_POINT_THRESHOLD = 8               # ignore jitter (pixels)
MIN_MOVEMENT_THRESHOLD = 40              # overall movement to count as "moved"

# Controller
DEADZONE = 0.18
MAX_SPEED = 4000
Y_BUTTON = 3
LT_AXIS = 5
RT_AXIS = 4

# Serial writer
_SER_LOCK = threading.Lock()

def send(ser, cmd):
    with _SER_LOCK:
        ser.write((cmd + "\n").encode())
    print(f" --> {cmd}")

# Shared data 
class GameUserData(app_callback_class):
    def __init__(self):
        super().__init__()
        self.tracks  = {}   # track_id -> {start_pts, end_pts, last_bbox}
        self.frame_w = None
        self.frame_h = None

        # tracking-phase state
        self.mode = 'scan'
        self.target_id = None
        self.accum_steps = 0
        self._last_cmd_t = 0.0
        self.ser = None

    def reset(self):
        with self._lock:
            self.tracks  = {}
            self.frame_w = None
            self.frame_h = None
            self.mode = 'scan'
            self.target_id = None
            self.accum_steps = 0
            self._last_cmd_t = 0.0

    # scan-phase updates
    def update(self, track_id, points, bbox, width, height):
        with self._lock:
            self.frame_w = width
            self.frame_h = height
            curr = [(p.x(), p.y()) for p in points]
            if track_id not in self.tracks:
                self.tracks[track_id] = {
                    'start_pts': curr,
                    'end_pts': curr,
                    'last_bbox': bbox,
                }
                return
            # update final pose continuously
            self.tracks[track_id]['end_pts'] = curr
            self.tracks[track_id]['last_bbox'] = bbox

    def compute_movement(self, start, end):
        total = 0.0
        count = 0

        for i in STABLE_KEYPOINTS:
            if i >= len(start) or i >= len(end):
                continue

            sx, sy = start[i]
            ex, ey = end[i]

            dx = (ex - sx) * self.frame_w
            dy = (ey - sy) * self.frame_h
            dist = (dx * dx + dy * dy) ** 0.5

            if dist > PER_POINT_THRESHOLD:
                total += dist
                count += 1

        return total / count if count > 0 else 0.0

    def most_moved(self):
        with self._lock:
            if not self.tracks:
                return None, None

            best_id = None
            best_move = 0.0

            for tid, data in self.tracks.items():
                start = data['start_pts']
                end   = data['end_pts']

                move = self.compute_movement(start, end)

                if move > best_move:
                    best_move = move
                    best_id = tid

            if best_id is None or best_move < MIN_MOVEMENT_THRESHOLD:
                return None, None

            return best_id, self.tracks[best_id]['last_bbox']

    def track_update(self, bbox, width, height):
        now = time.time()
        cx_norm = bbox.xmin() + bbox.width() / 2.0
        cx_px = cx_norm * width
        offset_px = cx_px - (width / 2.0)

        with self._lock:
            self.frame_w = width
            if now - self._last_cmd_t < TRACK_CMD_INTERVAL:
                return
            self._last_cmd_t = now
            if abs(offset_px) < CENTRE_TOL_PX:
                return
            delta = int(offset_px * STEPS_PER_PIXEL)
            delta = max(-MAX_PAN_STEPS, min(MAX_PAN_STEPS, delta))
            self.accum_steps += delta
            ser = self.ser

        if ser is not None:
            try:
                with _SER_LOCK:
                    ser.write(f"GOTO_REL {delta} 0\n".encode())
            except Exception as e:
                print("[track] serial: {e}")

            
# GStreamer callback 
def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    try:
        _, width, height = get_caps_from_pad(pad)
        if width is None or height is None:
            return Gst.PadProbeReturn.OK

        roi        = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

        with user_data._lock:
            mode = user_data.mode
            target_id = user_data.target_id

        for det in detections:
            if det.get_label() != "person":
                continue

            track = det.get_objects_typed(hailo.HAILO_UNIQUE_ID)
            if len(track) != 1:
                continue

            tid  = track[0].get_id()

            if mode == 'scan':
                landmarks = det.get_objects_typed(hailo.HAILO_LANDMARKS)
                if not landmarks:
                    continue
                points = landmarks(0).get_points()
                if len(points) == 0:
                    continue
                user_data.update(tid, points, det.get_bbox(), width, height)
            elif mode == 'track' and tid == target_id:
                user_data.track_update(det.getbbox(), width, height)

    except Exception as e:
        print(f"[callback] {e}")

    return Gst.PadProbeReturn.OK


# Red-light + tracking cycle
def run_red_scan(ser, user_data):
    project_root = Path(__file__).resolve().parent
    os.environ["HAILO_ENV_FILE"] = str(project_root / ".env")
    sys.argv = [sys.argv[0], "--input", "rpi"]

    user_data.reset()
    user_data.ser = ser

    app = GStreamerPoseEstimationApp(app_callback, user_data)

    red_duration = random.randint(RED_MIN_S, RED_MAX_S)
    print(f"[red] scanning for {red_duration}s")


    def _quit():
        try:
            app.loop.quit()
        except Exception:
            pass
        return False
    
    def end_scan():
        tid, bbox = user_data.most_moved()
        if tid is None or bbox is None:
            print("[judge] No significant movement.")
            _quit()
            return False
        
        print(f"[judge] Player {tid} moved most - entering Track phase.")
        # seed aim with the last scan bbox
        with user_data._lock:
            user_data.mode = 'track'
            user_data.target_id = tid
            user_data.accum_steps = 0

        cx_norm = bbox.xmin() + bbox.width() / 2.0
        cx_px = cx_norm * (user_data.frame_w or 0)
        offset_px = cx_px - ((user_data.frame_w or 0) / 2.0)
        seed = int(offset_px * STEPS_PER_PIXEL)
        seed = max(-MAX_PAN_STEPS, min(MAX_PAN_STEPS, seed))
        if seed != 0:
            send(ser, f"GOTO_REL {seed} 0")
            with user_data._lock:
                user_data.accum_steps += seed

        GLib.timeout_add(int(TRACK_DURATION_S * 1000), fire_laser)
        return False
    
    def fire_laser():
        send(ser, "L1")
        GLib.timeout_add(int(TRACK_DURATION_S * 1000), stop_laser)
        return False
    
    def stop_laser():
        send(ser, "L0")
        with user_data._lock:
            back = -user_data.accum_steps
            user_data.mode = 'scan'
        if back != 0:
            send(ser, f"GOTO_REL {back} 0")
        GLib.timeout_add(int(RETURN_SETTLE_S * 1000), _quit)
        return False
    
    GLib.timeout_add_seconds(red_duration, end_scan)
    app.run()

# Manual mode until Y press
def _dz(v):
    if abs(v) < DEADZONE:
        return 0.0
    return (v-DEADZONE if v > 0 else v + DEADZONE) / (1 - DEADZONE)

def manual_mode_until_y(ser):
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise SystemExit("No controller detected. pair it first.")
    js = pygame.joystick.Joystick(0)
    js.init()
    print("MANUAL mode active - press Y to start automated game.")

    laser_on = False
    water_on = False

    try:
        while True:
            pygame.event.pump()

            if js.get_button(Y_BUTTON):
                send(ser, "SPD 0 0")
                if laser_on:
                    send(ser, "L0")
                if water_on:
                    send(ser, "S0")
                print("[Y] pressed starting automated game")
                return
            
            pan = _dz(js.get_axis(0))
            tilt = _dz(js.get_axis(1))
            pan_s = int((pan ** 3) * MAX_SPEED)
            tilt_s = int((tilt ** 3) * MAX_SPEED)
            with _SER_LOCK:
                ser.write(f"SPD {pan_s} {tilt_s}\n".encode())

            lt = (js.get_axis(LT_AXIS) + 1) / 2
            rt = (js.get_axis(RT_AXIS) + 1) / 2
            if lt > 0.2 and not laser_on:
                send(ser, "L1"); laser_on = True
            elif lt <= 0.2 and laser_on:
                send(ser, "L0"); laser_on = False
            if rt > 0.2 and not water_on:
                send(ser, "S1"); water_on = True
            elif rt <= 0.2 and water_on:
                send(ser, "S0"); water_on = False

            time.sleep(0.03)
    except KeyboardInterrupt:
        send(ser, "SPD 0 0")
        raise



# Main 
def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        raise SystemExit(f"Cannot open {SERIAL_PORT}: {e}")

    manual_mode_until_y(ser)

    user_data = GameUserData()

    try:
        while True:
                green_duration = random.randint(GREEN_MIN_S, GREEN_MAX_S)
                print(f"\n GREEN LIGHT ({green_duration}s)")
                send(ser, "LED_GREEN")
                send(ser, "SPD 0 0")
                time.sleep(green_duration)

                print("\n RED LIGHT")
                send(ser, "LED_red")
                send(ser, "SPD 0 0")
                run_red_scan(ser, user_data)

    except KeyboardInterrupt:
        send(ser, "SPD 0 0")
        send(ser, "L0")
        send(ser, "S0")
        send(ser, "LED_OFF")
        print("\nStopped.")


if __name__ == "__main__":
    main()