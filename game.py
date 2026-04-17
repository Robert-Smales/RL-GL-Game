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
import threading
from pathlib import Path

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import serial
import hailo
from hailo_apps_infra.hailo_rpi_common import get_caps_from_pad, app_callback_class

sys.path.insert(0, str(Path(__file__).parent / 'pose_estimation'))
from pose_pipeline import GStreamerPoseEstimationApp

# ── Config ────────────────────────────────────────────────────────────────────
SERIAL_PORT        = "/dev/ttyUSB0"
BAUD_RATE          = 115200
RED_LIGHT_DURATION = 12

STEPS_PER_PIXEL    = 3.0
MAX_PAN_STEPS      = 10000

# Movement tuning
STABLE_KEYPOINTS       = [5, 6, 11, 12]  # shoulders + hips
PER_POINT_THRESHOLD    = 8               # ignore jitter (pixels)
MIN_MOVEMENT_THRESHOLD = 40              # overall movement to count as "moved"

# ── Shared data ───────────────────────────────────────────────────────────────
class GameUserData(app_callback_class):
    def __init__(self):
        super().__init__()
        self.tracks  = {}   # track_id -> {start_pts, end_pts, last_bbox}
        self.frame_w = None
        self.frame_h = None
        self._lock   = threading.Lock()

    def reset(self):
        with self._lock:
            self.tracks  = {}
            self.frame_w = None
            self.frame_h = None

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


# ── GStreamer callback ─────────────────────────────────────────────────────────
def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    try:
        format, width, height = get_caps_from_pad(pad)
        if width is None or height is None:
            return Gst.PadProbeReturn.OK

        roi        = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

        for det in detections:
            if det.get_label() != "person":
                continue

            track = det.get_objects_typed(hailo.HAILO_UNIQUE_ID)
            if len(track) != 1:
                continue

            track_id  = track[0].get_id()

            landmarks = det.get_objects_typed(hailo.HAILO_LANDMARKS)
            if not landmarks:
                continue

            points = landmarks[0].get_points()
            if len(points) == 0:
                continue

            user_data.update(track_id, points, det.get_bbox(), width, height)

    except Exception as e:
        print(f"[callback] {e}")

    return Gst.PadProbeReturn.OK


# ── Pipeline runner ────────────────────────────────────────────────────────────
def run_scan(user_data, duration):
    project_root = Path(__file__).resolve().parent
    os.environ["HAILO_ENV_FILE"] = str(project_root / ".env")

    sys.argv = [sys.argv[0], "--input", "rpi"]

    app = GStreamerPoseEstimationApp(app_callback, user_data)

    def _stop():
        try:
            app.loop.quit()
        except Exception:
            pass
        return False

    GLib.timeout_add_seconds(duration, _stop)
    app.run()


# ── Serial helpers ─────────────────────────────────────────────────────────────
def send(ser, cmd):
    ser.write((cmd + "\n").encode())
    print(f"  → {cmd}")


# ── Aim logic ──────────────────────────────────────────────────────────────────
def aim_at_player(ser, bbox, frame_w):
    if bbox is None or frame_w is None:
        print("[aim] No target.")
        return

    cx_norm     = bbox.xmin() + bbox.width() / 2.0
    cx_px       = cx_norm * frame_w
    offset_px   = cx_px - (frame_w / 2.0)

    delta_steps = int(offset_px * STEPS_PER_PIXEL)
    delta_steps = max(-MAX_PAN_STEPS, min(MAX_PAN_STEPS, delta_steps))

    send(ser, f"GOTO_REL {delta_steps} 0")
    print(f"[aim] offset {offset_px:.1f}px → {delta_steps} steps")


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        raise SystemExit(f"Cannot open {SERIAL_PORT}: {e}")

    user_data = GameUserData()

    print(f"Game ready. Press Enter to trigger RED LIGHT ({RED_LIGHT_DURATION}s scan).")
    print("Ctrl-C to quit.\n")

    try:
        while True:
            input("[ waiting — press Enter for RED LIGHT ]")

            # ── RED LIGHT ──────────────────────────────────────────────────────
            print("\n=== RED LIGHT ===")
            send(ser, "LED_RED")
            send(ser, "SPD 0 0")

            user_data.reset()
            run_scan(user_data, RED_LIGHT_DURATION)

            # ── JUDGE ──────────────────────────────────────────────────────────
            tid, bbox = user_data.most_moved()

            if tid is not None:
                print(f"[judge] Player {tid} moved most")
                aim_at_player(ser, bbox, user_data.frame_w)
            else:
                print("[judge] No significant movement detected.")

            # ── GREEN LIGHT ────────────────────────────────────────────────────
            print("\n=== GREEN LIGHT ===")
            send(ser, "LED_GREEN")

    except KeyboardInterrupt:
        send(ser, "SPD 0 0")
        send(ser, "LED_OFF")
        print("\nStopped.")


if __name__ == "__main__":
    main()