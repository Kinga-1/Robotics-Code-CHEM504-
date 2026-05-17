"""
colour_monitor.py
==================
Fixed overhead camera monitors vial colour at the filming position.
Signals the robot when colour transitions occur.

Colour sequence tracked:
    GREEN  →  RED  →  YELLOW  (triggers shake + return to filming pos)

Usage in master script:
    from colour_monitor import ColourMonitor, ColourState

    monitor = ColourMonitor(camera_index=1)   # 1 = second camera
    monitor.wait_for_transition(ColourState.GREEN, ColourState.RED,   timeout_mins=30)
    monitor.wait_for_transition(ColourState.RED,   ColourState.YELLOW, timeout_mins=30)
    monitor.release()

Design:
    - Analyses a region-of-interest (ROI) in the centre of the frame
      so the white background doesn't interfere.
    - Colour classified by HSV hue ranges — robust to brightness variation.
    - Requires N consecutive frames to agree before confirming a transition
      (avoids single-frame false triggers).
    - Timeout raises ColourTimeoutError so master script can handle it cleanly.
    - Live preview window shows ROI box, current colour label, and progress bar.
"""

import cv2
import numpy as np
import time
import threading
from enum import Enum, auto
from collections import deque


# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────

# How many consecutive frames must agree on the new colour before confirming.
# At ~30 fps, 45 frames ≈ 1.5 seconds of stable colour.
CONFIRMATION_FRAMES = 45

# Rolling window size for colour voting
# Each frame votes on what colour it sees, and the majority vote wins. 
# So a single weird frame doesn't count for much.
WINDOW_SIZE = 60

# ROI as fraction of frame: (x_start, y_start, x_end, y_end) — 0.0 to 1.0
# Crops to centre third of frame to focus on vial, ignore background edges.
ROI = (0.45, 0.35, 0.55, 0.65) # exactly vial size :D

# Camera index for the fixed overhead/side camera (usually 1 if wrist cam is 0)
DEFAULT_CAMERA_INDEX = 0

# HSV colour ranges  [lower_bound, upper_bound]
# Hue is 0-179 in OpenCV, Saturation/Value 0-255
# Adjust these after running the calibration tool below if needed.

# HSV_RANGES = {
#     "GREEN":  [(62, 106, 31), (120, 255, 255)],
#     "RED":    [(3, 93, 54), (17, 255, 255)],   # red wraps; second range below
#     "RED2":   [(172, 93, 54), (179, 255, 255)],   # upper red hue wrap-around
#     "YELLOW": [(41, 87, 69), (50, 255, 255)],
# }

HSV_RANGES = {
    "GREEN":  [(25, 60, 30),  (50, 255, 180)],   # olive/dark yellow-green
    "RED":    [(0,  100, 50), (10, 255, 255)],
    "RED2":   [(165, 100, 50),(179, 255, 255)],
    "YELLOW": [(15, 100, 100),(30, 255, 255)],    # amber/golden
}

# Minimum fraction of ROI pixels that must match a colour to classify it
COLOUR_DOMINANCE_THRESHOLD = 0.25   # 25% of ROI pixels


# ──────────────────────────────────────────────────────────────────────────────
# COLOUR STATE ENUM
# ──────────────────────────────────────────────────────────────────────────────

class ColourState(Enum):
    UNKNOWN = auto()
    GREEN   = auto()
    RED     = auto()
    YELLOW  = auto()


class ColourTimeoutError(Exception):
    # raised when a colour transition does not occur within the timeout.
    pass


# ──────────────────────────────────────────────────────────────────────────────
# COLOUR CLASSIFIER
# ──────────────────────────────────────────────────────────────────────────────

def classify_colour(hsv_roi):
    # classify the dominant colour in an HSV ROI image and return its state and confidence.
    total_pixels = hsv_roi.shape[0] * hsv_roi.shape[1]
    if total_pixels == 0:
        return ColourState.UNKNOWN, 0.0

    scores = {}

    for colour in ["GREEN", "YELLOW"]:
        lo = np.array(HSV_RANGES[colour][0], dtype=np.uint8)
        hi = np.array(HSV_RANGES[colour][1], dtype=np.uint8)
        mask = cv2.inRange(hsv_roi, lo, hi)
        scores[colour] = np.count_nonzero(mask) / total_pixels

    # Red wraps around hue — combine both ranges
    lo1 = np.array(HSV_RANGES["RED"][0],  dtype=np.uint8)
    hi1 = np.array(HSV_RANGES["RED"][1],  dtype=np.uint8)
    lo2 = np.array(HSV_RANGES["RED2"][0], dtype=np.uint8)
    hi2 = np.array(HSV_RANGES["RED2"][1], dtype=np.uint8)
    red_mask = cv2.inRange(hsv_roi, lo1, hi1) | cv2.inRange(hsv_roi, lo2, hi2)
    scores["RED"] = np.count_nonzero(red_mask) / total_pixels

    best_colour = max(scores, key=scores.get)
    best_score  = scores[best_colour]

    if best_score < COLOUR_DOMINANCE_THRESHOLD:
        return ColourState.UNKNOWN, best_score

    return ColourState[best_colour], best_score


# ──────────────────────────────────────────────────────────────────────────────
# MAIN MONITOR CLASS
# ──────────────────────────────────────────────────────────────────────────────

class ColourMonitor:
    # monitors vial colour from a fixed camera and waits for specified transitions.
    def __init__(self, camera_index=DEFAULT_CAMERA_INDEX, show_preview=True, monitor_id=None, monitor_name=None, status_callback=None):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Cannot open camera index {camera_index}. "
                "Check it is connected and not in use by another process."
            )
        self.show_preview = show_preview
        self.monitor_id = monitor_id
        self.monitor_name = monitor_name or f"Camera {camera_index}"
        self.status_callback = status_callback
        self.colour_window = deque(maxlen=WINDOW_SIZE)
        self.latest_frame = None
        self.lock = threading.Lock()
        self._running = True
        self.frame_grabber_thread = threading.Thread(target=self._frame_grabber_loop, daemon=True)
        self.frame_grabber_thread.start()

        self.ui_state = {
            "voted_colour": ColourState.UNKNOWN,
            "waiting_for": None,
            "elapsed_s": 0,
            "timeout_s": 1,
            "confirm_streak": 0,
            "confirmation_frames": CONFIRMATION_FRAMES,
            "is_waiting": False,
            "avg_rgb": (0, 0, 0),
            "confidence": 0.0,
        }
        print(f"[ColourMonitor] Camera {camera_index} opened.")

    def _get_roi(self, frame):
        # crop frame to ROI and return BGR, HSV versions.
        h, w = frame.shape[:2]
        x1 = int(ROI[0] * w);  y1 = int(ROI[1] * h)
        x2 = int(ROI[2] * w);  y2 = int(ROI[3] * h)
        roi_bgr = frame[y1:y2, x1:x2]
        if roi_bgr.size == 0:
            # Return empty arrays and original coords if crop is invalid
            return np.zeros((1,1,3), dtype=np.uint8), np.zeros((1,1,3), dtype=np.uint8), (x1, y1, x2, y2)
        roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        return roi_bgr, roi_hsv, (x1, y1, x2, y2)

    def _frame_grabber_loop(self):
        # continuously grabs frames from the camera in a background thread.
        while self._running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame
            else:
                # Avoid busy-waiting if the camera disconnects or reaches end of file
                time.sleep(0.05)

    def _current_colour(self):
        # analyse the latest frame, classify colour, and update the rolling window.
        # returns the majority-vote colour over the window.
        with self.lock:
            if self.latest_frame is None:
                self.ui_state.update({
                    "voted_colour": ColourState.UNKNOWN,
                    "confidence": 0.0,
                    "avg_rgb": (0, 0, 0)
                })
                return ColourState.UNKNOWN, None, None, None
            frame = self.latest_frame.copy()

        roi_bgr, roi_hsv, roi_coords = self._get_roi(frame)
        
        if roi_bgr.size > 0:
            avg_bgr = np.mean(roi_bgr, axis=(0, 1))
            avg_rgb = (int(avg_bgr[2]), int(avg_bgr[1]), int(avg_bgr[0]))
            colour, score = classify_colour(roi_hsv)
        else:
            avg_rgb = (0,0,0)
            colour = ColourState.UNKNOWN
            score = 0.0

        self.colour_window.append(colour)

        # Majority vote over window
        if len(self.colour_window) > 0:
            counts = {}
            for c in self.colour_window:
                counts[c] = counts.get(c, 0) + 1
            voted = max(counts, key=counts.get)
        else:
            voted = ColourState.UNKNOWN

        self.ui_state["voted_colour"] = voted
        self.ui_state["confidence"] = score
        self.ui_state["avg_rgb"] = avg_rgb

        return voted, frame, roi_coords, colour

    def _draw_preview(self, frame, roi_coords, raw_colour, voted_colour,
                      waiting_for, elapsed_s, timeout_s, confirm_streak):
        # annotate and show the live preview window or update the UI status panel.
        x1, y1, x2, y2 = roi_coords

        pct = min(elapsed_s / timeout_s, 1.0)
        mins_left = max(0, (timeout_s - elapsed_s) / 60)

        status_text = (
            f"{self.monitor_name}\n"
            f"Waiting for: {waiting_for.name}\n"
            f"Current: {voted_colour.name}\n"
            f"Timeout: {mins_left:.1f} min left\n"
            f"Streak: {confirm_streak}/{CONFIRMATION_FRAMES}"
        )

        if self.status_callback is not None and self.monitor_id is not None:
            self.status_callback(self.monitor_id, status_text)
            return

        # Fallback to OpenCV preview when no UI callback is registered.
        # ROI box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)

        colour_bgr = {
            ColourState.GREEN:   (0, 200, 0),
            ColourState.RED:     (0, 0, 220),
            ColourState.YELLOW:  (0, 220, 220),
            ColourState.UNKNOWN: (120, 120, 120),
        }
        chip_colour = colour_bgr.get(voted_colour, (120, 120, 120))
        cv2.rectangle(frame, (10, 10), (300, 130), (30, 30, 30), -1)
        cv2.rectangle(frame, (10, 10), (300, 130), chip_colour, 2)

        cv2.putText(frame, f"Colour: {voted_colour.name}",
                    (18, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.8, chip_colour, 2)
        cv2.putText(frame, f"Waiting for: {waiting_for.name}",
                    (18, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 1)

        bar_w = 270
        cv2.rectangle(frame, (18, 85), (18 + bar_w, 100), (60, 60, 60), -1)
        bar_colour = (0, 200, 0) if pct < 0.75 else (0, 100, 255)
        cv2.rectangle(frame, (18, 85), (18 + int(bar_w * pct), 100), bar_colour, -1)

        cv2.putText(frame, f"Timeout in {mins_left:.1f} min  |  streak: {confirm_streak}/{CONFIRMATION_FRAMES}",
                    (18, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        cv2.imshow("Colour Monitor", frame)
        cv2.waitKey(1)

    def current_colour_snapshot(self):
        # return the current voted colour without waiting. Useful for diagnostics.
        result = self._current_colour()
        return result[0]

    def get_frame(self):
        # return the latest cached frame for UI display.
        with self.lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def wait_for_transition(self, from_colour: ColourState, to_colour: ColourState,
                             timeout_mins: float = 30.0):
        # block until the vial colour transitions from from_colour → to_colour.
        timeout_s = timeout_mins * 60
        start_time = time.time()
        confirm_streak = 0
        self.colour_window.clear()

        print(f"\n[ColourMonitor] Waiting: {from_colour.name} → {to_colour.name}  "
              f"(timeout {timeout_mins:.0f} min)")

        try:
            self.ui_state.update({
                "is_waiting": True,
                "timeout_s": timeout_s,
                "confirmation_frames": CONFIRMATION_FRAMES,
            })

            # Phase 1: confirm we are currently seeing from_colour
            print(f"[ColourMonitor] Phase 1 — confirming starting colour is {from_colour.name} ...")
            while True:
                if not self._running:
                    print("[ColourMonitor] Wait aborted during phase 1.")
                    return

                elapsed = time.time() - start_time
                if elapsed > timeout_s:
                    raise ColourTimeoutError(
                        f"Timeout waiting to confirm starting colour {from_colour.name}. "
                        f"Check vial is in filming position and lighting is consistent."
                    )
                result = self._current_colour()
                voted, frame, roi_coords, raw = result[0], result[1], result[2], result[3]

                if voted == from_colour:
                    confirm_streak += 1
                else:
                    confirm_streak = 0

                self.ui_state.update({
                    "waiting_for": from_colour, "elapsed_s": elapsed, "confirm_streak": confirm_streak
                })

                if self.show_preview and frame is not None:
                    self._draw_preview(frame, roi_coords, raw, voted,
                                       from_colour, elapsed, timeout_s, confirm_streak)

                if confirm_streak >= CONFIRMATION_FRAMES:
                    print(f"[ColourMonitor] Starting colour {from_colour.name} confirmed.")
                    break

            # Phase 2: wait for to_colour
            confirm_streak = 0
            print(f"[ColourMonitor] Phase 2 — watching for {to_colour.name} ...")
            while True:
                if not self._running:
                    print("[ColourMonitor] Wait aborted during phase 2.")
                    return

                elapsed = time.time() - start_time
                if elapsed > timeout_s:
                    raise ColourTimeoutError(
                        f"Timeout after {timeout_mins:.0f} min waiting for {to_colour.name}. "
                        f"Reaction may have stalled. Robot has been stopped safely."
                    )

                result = self._current_colour()
                voted, frame, roi_coords, raw = result[0], result[1], result[2], result[3]

                if voted == to_colour:
                    confirm_streak += 1
                else:
                    if confirm_streak > 0:
                        print(f"[ColourMonitor] Streak broken at {confirm_streak} — resetting.")
                    confirm_streak = 0

                self.ui_state.update({
                    "waiting_for": to_colour, "elapsed_s": elapsed, "confirm_streak": confirm_streak
                })

                if self.show_preview and frame is not None:
                    self._draw_preview(frame, roi_coords, raw, voted,
                                       to_colour, elapsed, timeout_s, confirm_streak)

                if confirm_streak >= CONFIRMATION_FRAMES:
                    elapsed_mins = elapsed / 60
                    print(f"[ColourMonitor] ✓ {to_colour.name} confirmed after "
                          f"{elapsed_mins:.1f} min  ({confirm_streak} consecutive frames).")
                    return   # signal caller to proceed
        finally:
            self.ui_state["is_waiting"] = False
                
    def release(self):
        # release camera and close preview window.
        self._running = False
        if hasattr(self, 'frame_grabber_thread') and self.frame_grabber_thread.is_alive():
            self.frame_grabber_thread.join(timeout=1.0)

        self.cap.release()
        cv2.destroyAllWindows()
        print("[ColourMonitor] Camera released.")


# ──────────────────────────────────────────────────────────────────────────────
# HSV CALIBRATION HELPER
# ──────────────────────────────────────────────────────────────────────────────

def run_hsv_calibration(camera_index=DEFAULT_CAMERA_INDEX):
    # interactive tool to find the right HSV values for your specific vial + lighting.
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[Error] Cannot open camera {camera_index}")
        return

    clicked_hsv = [None]

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            frame = param[0]
            if frame is not None:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                h, s, v = hsv[y, x]
                clicked_hsv[0] = (h, s, v)
                print(f"  Clicked pixel HSV: H={h}  S={s}  V={v}  "
                      f"→ lower bound hint: ({max(0,h-15)}, {max(0,s-40)}, {max(0,v-40)}), "
                      f"upper: ({min(179,h+15)}, 255, 255)")

    frame_holder = [None]
    cv2.namedWindow("HSV Calibration — click on vial")
    cv2.setMouseCallback("HSV Calibration — click on vial",
                         on_click, frame_holder)

    print("\nHSV Calibration Tool")
    print("Click on the vial at each colour stage to read HSV values.")
    print("Press 'q' to quit.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_holder[0] = frame.copy()

        # Draw ROI
        h, w = frame.shape[:2]
        x1,y1 = int(ROI[0]*w), int(ROI[1]*h)
        x2,y2 = int(ROI[2]*w), int(ROI[3]*h)
        cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,255), 2)
        cv2.putText(frame, "Click on vial to read HSV  |  q = quit",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200,200,200), 2)
        if clicked_hsv[0]:
            h_v,s_v,v_v = clicked_hsv[0]
            cv2.putText(frame, f"Last click: H={h_v} S={s_v} V={v_v}",
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,255), 2)

        cv2.imshow("HSV Calibration — click on vial", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibrate", action="store_true",
                        help="Run interactive HSV calibration tool")
    parser.add_argument("--camera", type=int, default=DEFAULT_CAMERA_INDEX,
                        help="Camera index (default: 1)")
    args = parser.parse_args()
    if args.calibrate:
        run_hsv_calibration(args.camera)
    else:
        # Demo: just show live colour detection
        mon = ColourMonitor(camera_index=args.camera)
        print("Live demo — press q in preview window to quit.")
        try:
            while True:
                c = mon.current_colour_snapshot()
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            mon.release()
