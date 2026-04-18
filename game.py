# setup the virtual environment

import os as _os
import sys as _sys
from pathlib import Path as _Path

if _os.environ.get("RLGL_ENV_READY") != "1":
    _script_dir = _Path(__file__).resolve().parent
    _setup = _script_dir / "setup_env.sh"
    _venv_py = _script_dir / "venv_hailo_rpi5_examples" / "bin" / "python"
    _py = str(_venv_py) if _venv_py.exists() else _sys.executable
    if _setup.exists():
        _os.execvp("bash", [
            "bash", "-c", 
            f'source "{_setup}" >&2; RLGL_ENV_READY=1 exec "{_py}" "{__file__}" "$@"',
            "bash", *_sys.argv[1:],
        ])

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

GREEN_MIN_S = 5
GREEN_MAX_S = 7
RED_MIN_S = 5
RED_MAX_S = 7

TRACK_MAX_S = 4.0            # hard cap on how long we chase a target
LASER_DURATION_S = 1.0
RETURN_SETTLE_S = 1.5
ON_TARGET_FRAMES_REQUIRED = 3  # consecutive centred frames before firing

STEPS_PER_PIXEL = 2.0
MAX_PAN_STEPS = 5000
CENTRE_TOL_PX = 15
TRACK_CMD_INTERVAL = 0.15
PAN_INVERT = True           # flip if pan moves the wrong way
TILT_INVERT = True           # flip if tilt moves the wrong way
POLL_Y_MS = 50               # how often to sample Y during auto mode

# Movement tuning
STABLE_KEYPOINTS = [5, 6, 11, 12]  # shoulders + hips
PER_POINT_THRESHOLD = 8               # ignore jitter (pixels)
MIN_MOVEMENT_THRESHOLD = 40              # overall movement to count as "moved"

# Controller
DEADZONE = 0.18
MAX_SPEED = 4000
Y_BUTTON = 3
A_BUTTON = 0
B_BUTTON = 1
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
        self._lock = threading.Lock()
        super().__init__()
        self.tracks  = {}   # track_id -> {start_pts, end_pts, last_bbox}
        self.frame_w = None
        self.frame_h = None

        # tracking-phase state
        self.mode = 'scan'
        self.target_id = None
        self.accum_pan = 0
        self.accum_tilt = 0
        self.on_target_frames = 0
        self._last_cmd_t = 0.0
        self.ser = None
        # diagnostics
        self.track_updates = 0
        self.track_ids_seen = set()

    def reset(self):
        with self._lock:
            self.tracks  = {}
            self.frame_w = None
            self.frame_h = None
            self.mode = 'scan'
            self.target_id = None
            self.accum_pan = 0
            self.accum_tilt = 0
            self.on_target_frames = 0
            self._last_cmd_t = 0.0
            self.track_updates = 0
            self.track_ids_seen = set()

    # scan-phase updates
    def update(self, track_id, points, bbox, width, height):
        with self._lock:
            self.frame_w = width
            self.frame_h = height
            curr = [(p.x(), p.y()) for p in points]

            if track_id not in self.tracks:
                self.tracks[track_id] = {
                    'start_pts': curr,
                    'peak_per_point': [0.0] * len(curr),
                    'last_bbox': bbox,
                }
                return

            data = self.tracks[track_id]
            start = data['start_pts']
            peak = data['peak_per_point']

            if len(peak) < len(curr):
                peak.extend([0.0] * (len(curr) - len(peak)))

            for i in range(min(len(start), len(curr))):
                sx, sy = start[i]
                cx, cy = curr[i]
                dx = (cx - sx) * width
                dy = (cy - sy) * height
                dist = (dx * dx + dy * dy) ** 0.5
                if dist > peak[i]:
                    peak[i] = dist

            data['last_bbox'] = bbox

    def score_peaks(self, peak_per_point):
        total = 0.0
        count = 0
        for i in STABLE_KEYPOINTS:
            if i >= len(peak_per_point):
                continue
            if peak_per_point[i] > PER_POINT_THRESHOLD:
                total += peak_per_point[i]
                count += 1
        return total / count if count > 0 else 0.0

    def most_moved(self):
        with self._lock:
            if not self.tracks:
                return None, None

            best_id = None
            best_move = 0.0

            for tid, data in self.tracks.items():
                move = self.score_peaks(data['peak_per_point'])
                if move > best_move:
                    best_move = move
                    best_id = tid

            if best_id is None or best_move < MIN_MOVEMENT_THRESHOLD:
                return None, None

            return best_id, self.tracks[best_id]['last_bbox']

    def track_update(self, bbox, width, height):
        now = time.time()
        cx_px = (bbox.xmin() + bbox.width() / 2.0) * width
        cy_px = (bbox.ymin() + bbox.height() / 2.0) * height
        offset_x = cx_px - (width / 2.0)
        offset_y = cy_px - (height / 2.0)

        delta_pan = 0
        delta_tilt = 0
        ser = None
        should_send = False

        with self._lock:
            self.frame_w = width
            self.frame_h = height
            self.track_updates += 1

            on_tgt = abs(offset_x) < CENTRE_TOL_PX and abs(offset_y) < CENTRE_TOL_PX
            if on_tgt:
                self.on_target_frames += 1
            else:
                self.on_target_frames = 0

            if now - self._last_cmd_t < TRACK_CMD_INTERVAL:
                return
            self._last_cmd_t = now
            if on_tgt:
                return

            raw_pan = int(offset_x * STEPS_PER_PIXEL)
            raw_tilt = int(offset_y * STEPS_PER_PIXEL)
            delta_pan = -raw_pan if PAN_INVERT else raw_pan
            delta_tilt = -raw_tilt if TILT_INVERT else raw_tilt
            delta_pan = max(-MAX_PAN_STEPS, min(MAX_PAN_STEPS, delta_pan))
            delta_tilt = max(-MAX_PAN_STEPS, min(MAX_PAN_STEPS, delta_tilt))
            self.accum_pan += delta_pan
            self.accum_tilt += delta_tilt
            ser = self.ser
            should_send = True

        if should_send and ser is not None:
            print(f"[track] off=({offset_x:+.0f},{offset_y:+.0f}) d=({delta_pan:+d},{delta_tilt:+d})")
            try:
                with _SER_LOCK:
                    ser.write(f"GOTO_REL {delta_pan} {delta_tilt}\n".encode())
            except Exception as e:
                print(f"[track] serial: {e}")

            
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
                points = landmarks[0].get_points()
                if len(points) == 0:
                    continue
                user_data.update(tid, points, det.get_bbox(), width, height)
            elif mode == 'track':
                with user_data._lock:
                    user_data.track_ids_seen.add(tid)
                if tid == target_id:
                    user_data.track_update(det.get_bbox(), width, height)

    except Exception as e:
        import traceback
        print(f"[callback] {e}")
        traceback.print_exc()
    return Gst.PadProbeReturn.OK


# Abort flag used to interrupt automated mode with Y
class Abort:
    def __init__(self):
        self._flag = False
    def reset(self):
        self._flag = False
    def trigger(self):
        self._flag = True
    @property
    def flag(self):
        return self._flag


# Red-light + tracking cycle
def run_red_scan(ser, user_data, js, abort):
    project_root = Path(__file__).resolve().parent
    os.environ["HAILO_ENV_FILE"] = str(project_root / ".env")
    sys.argv = [sys.argv[0], "--input", "rpi"]

    user_data.reset()
    user_data.ser = ser

    app = GStreamerPoseEstimationApp(app_callback, user_data)

    red_duration = random.randint(RED_MIN_S, RED_MAX_S)
    print(f"[red] scanning for {red_duration}s")

    state = {'scheduled': [], 'track_start': 0.0, 'aborted': False}

    def schedule(ms, func):
        sid = GLib.timeout_add(ms, func)
        state['scheduled'].append(sid)
        return sid

    def cancel_all_scheduled():
        for sid in state['scheduled']:
            try:
                GLib.source_remove(sid)
            except Exception:
                pass
        state['scheduled'].clear()

    def _quit():
        try:
            app.loop.quit()
        except Exception:
            pass
        return False

    def abort_now():
        if state['aborted']:
            return
        state['aborted'] = True
        cancel_all_scheduled()
        try:
            send(ser, "SPD 0 0")
            send(ser, "L0")
            send(ser, "S0")
        except Exception:
            pass
        _quit()

    def poll_y():
        if state['aborted']:
            return False
        pygame.event.pump()
        if js.get_button(Y_BUTTON):
            print("[Y] pressed -> aborting auto mode")
            abort.trigger()
            abort_now()
            return False
        return True

    def end_scan():
        if state['aborted']:
            return False
        tid, bbox = user_data.most_moved()
        if tid is None or bbox is None:
            print("[judge] No significant movement.")
            _quit()
            return False

        print(f"[judge] Player {tid} moved most - entering Track phase.")
        # Zero the motor origin so HOME recenters exactly regardless of how
        # many GOTO_REL commands we pile up during tracking.
        send(ser, "ZERO")
        with user_data._lock:
            user_data.mode = 'track'
            user_data.target_id = tid
            user_data.accum_pan = 0
            user_data.accum_tilt = 0
            user_data.on_target_frames = 0

        w = user_data.frame_w or 0
        h = user_data.frame_h or 0
        cx_px = (bbox.xmin() + bbox.width() / 2.0) * w
        cy_px = (bbox.ymin() + bbox.height() / 2.0) * h
        offset_x = cx_px - (w / 2.0)
        offset_y = cy_px - (h / 2.0)
        raw_pan = int(offset_x * STEPS_PER_PIXEL)
        raw_tilt = int(offset_y * STEPS_PER_PIXEL)
        seed_pan = -raw_pan if PAN_INVERT else raw_pan
        seed_tilt = -raw_tilt if TILT_INVERT else raw_tilt
        seed_pan = max(-MAX_PAN_STEPS, min(MAX_PAN_STEPS, seed_pan))
        seed_tilt = max(-MAX_PAN_STEPS, min(MAX_PAN_STEPS, seed_tilt))
        if seed_pan != 0 or seed_tilt != 0:
            send(ser, f"GOTO_REL {seed_pan} {seed_tilt}")
            with user_data._lock:
                user_data.accum_pan += seed_pan
                user_data.accum_tilt += seed_tilt

        state['track_start'] = time.time()
        schedule(100, wait_on_target)
        return False

    def wait_on_target():
        if state['aborted']:
            return False
        with user_data._lock:
            ready = user_data.on_target_frames >= ON_TARGET_FRAMES_REQUIRED
            updates = user_data.track_updates
            ids_seen = set(user_data.track_ids_seen)
            accum = (user_data.accum_pan, user_data.accum_tilt)
            on_f = user_data.on_target_frames
        elapsed = time.time() - state['track_start']
        if ready:
            print(f"[track] on target after {elapsed:.1f}s - firing")
            fire_laser()
            return False
        if elapsed >= TRACK_MAX_S:
            print(f"[track] TIMEOUT after {elapsed:.1f}s | "
                  f"target_id={user_data.target_id} updates={updates} "
                  f"ids_seen={ids_seen} on_target={on_f} accum={accum}")
            fire_laser()
            return False
        return True

    def fire_laser():
        if state['aborted']:
            return False
        send(ser, "L1")
        schedule(int(LASER_DURATION_S * 1000), stop_laser)
        return False

    def stop_laser():
        if state['aborted']:
            return False
        send(ser, "L0")
        with user_data._lock:
            user_data.mode = 'scan'
        # HOME uses the motor's actual current position (zeroed at track start)
        send(ser, "HOME")
        schedule(int(RETURN_SETTLE_S * 1000), _quit)
        return False

    GLib.timeout_add(POLL_Y_MS, poll_y)
    schedule(red_duration * 1000, end_scan)
    # kept calling a sys.exit that stopped the whole python pipeline, means it can loop back to green light
    try:
        app.run()
    except SystemExit as e:
        if e.code not in (0, None):
            raise

# Manual mode until Y press
def _dz(v):
    if abs(v) < DEADZONE:
        return 0.0
    return (v-DEADZONE if v > 0 else v + DEADZONE) / (1 - DEADZONE)

def init_joystick():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise SystemExit("No controller detected. pair it first.")
    js = pygame.joystick.Joystick(0)
    js.init()
    return js

def wait_button_release(js, btn, timeout_s=2.0):
    end = time.time() + timeout_s
    while time.time() < end:
        pygame.event.pump()
        if not js.get_button(btn):
            return
        time.sleep(0.02)

def manual_mode_until_y(ser, js):
    print("MANUAL mode active - press Y to start automated game.")
    wait_button_release(js, Y_BUTTON)

    laser_on = False
    water_on = False
    a_last = 0
    b_last = 0
    led_red = False

    try:
        while True:
            pygame.event.pump()

            a_now = js.get_button(A_BUTTON)
            if a_now == 1 and a_last == 0:
                led_red = not led_red
                if led_red:
                    ser.write("LED_RED\n".encode())
                    print("LED RED")
                else:
                    ser.write("LED_GREEN\n".encode())
                    print("LED GREEN")
            a_last = a_now

            b_now = js.get_button(B_BUTTON)
            if b_now and not b_last:
                send(ser, "LED_OFF")
                led_red = False
            b_last = b_now

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
            tilt_s = int((-tilt ** 3) * MAX_SPEED)
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



# Sleep that aborts on Y press
def green_sleep(js, abort, duration):
    end = time.time() + duration
    while time.time() < end:
        pygame.event.pump()
        if js.get_button(Y_BUTTON):
            print("[Y] pressed -> aborting auto mode")
            abort.trigger()
            return
        time.sleep(0.05)


def run_auto_game(ser, user_data, js, abort):
    wait_button_release(js, Y_BUTTON)
    abort.reset()
    try:
        while not abort.flag:
            green_duration = random.randint(GREEN_MIN_S, GREEN_MAX_S)
            print(f"\n GREEN LIGHT ({green_duration}s)")
            send(ser, "LED_GREEN")
            send(ser, "SPD 0 0")
            green_sleep(js, abort, green_duration)
            if abort.flag:
                print("[auto] aborted during green"); break

            print("\n RED LIGHT")
            send(ser, "LED_RED")
            send(ser, "SPD 0 0")
            try:
                run_red_scan(ser, user_data, js, abort)
            except Exception as e:
                import traceback
                print(f"[auto] run_red_scan exception: {e}")
                traceback.print_exc()
                break
            print(f"[auto] red scan finished, abort={abort.flag}")
            if abort.flag:
                print("[auto] aborted during red"); break
    finally:
        print("[auto] exiting auto loop, cleaning up")
        # safety stop on every exit path
        try:
            send(ser, "SPD 0 0")
            send(ser, "L0")
            send(ser, "S0")
            send(ser, "LED_OFF")
        except Exception:
            pass


# Main
def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        raise SystemExit(f"Cannot open {SERIAL_PORT}: {e}")

    js = init_joystick()
    user_data = GameUserData()
    abort = Abort()

    try:
        while True:
            manual_mode_until_y(ser, js)
            run_auto_game(ser, user_data, js, abort)
            print("Returned to MANUAL mode.")
    except KeyboardInterrupt:
        send(ser, "SPD 0 0")
        send(ser, "L0")
        send(ser, "S0")
        send(ser, "LED_OFF")
        print("\nStopped.")


if __name__ == "__main__":
    main()