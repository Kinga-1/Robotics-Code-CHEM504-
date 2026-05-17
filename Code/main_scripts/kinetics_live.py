"""
kinetics_live.py
================
Extracted from simulation1.py and dashboard.py.

Replaces SimCamera with real OpenCV camera captures (cam1 / cam2).
Provides:
  - CameraCapture   : threaded frame grabber for a real camera index
  - detect_colour   : HSV-based colour classifier (green / red / yellow / none)
  - hsv_mean        : mean HSV of the centre ROI
  - KineticsEngine  : logs colour transitions, derives kinetic metrics
  - get_graph_data  : returns everything the dashboard graphs need in one call

Requirements:
    pip install numpy opencv-python scipy   # scipy optional but recommended
"""

import time
import threading
import collections
from datetime import datetime

import numpy as np

try:
    import cv2
    CV2 = True
except ImportError:
    CV2 = False
    raise ImportError("opencv-python is required for live camera capture.")

try:
    from scipy.signal import savgol_filter
    SCIPY = True
except ImportError:
    SCIPY = False


# ─── HSV colour ranges (ported directly from simulation1.py) ─────────────────

_HSV_RANGES = {
    "green":  [((35,  50,  50), (85,  255, 255))],
    "red":    [((0,   70,  50), (10,  255, 255)),
               ((160, 70,  50), (180, 255, 255))],
    "yellow": [((20,  80,  80), (35,  255, 255))],
}


# ─── Colour detection helpers ─────────────────────────────────────────────────

def detect_colour(frame_rgb: np.ndarray) -> tuple[str, dict]:
    """
    Classify the dominant colour in a frame.

    Uses OpenCV HSV masking (centre ROI, same ranges as simulation1.py).
    Returns (colour_str, confidence_dict) where colour_str is one of:
        'green' | 'red' | 'yellow' | 'none'
    and confidence_dict maps each colour to a 0-100 percentage score.
    """
    bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    sc  = {}
    for name, ranges in _HSV_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        sc[name] = int(mask.sum())

    best  = max(sc, key=sc.get)
    total = max(sum(sc.values()), 1)
    conf  = {k: round(v / total * 100, 1) for k, v in sc.items()}
    return (best if sc[best] > 300 else "none"), conf


def _rgb_scores(frame_rgb: np.ndarray) -> dict:
    # fallback scorer when OpenCV is unavailable (not used in live path).
    h, w = frame_rgb.shape[:2]
    roi  = frame_rgb[h//4:3*h//4, w//4:3*w//4]
    m    = roi.mean(axis=(0, 1))
    r, g, b = float(m[0]), float(m[1]), float(m[2])
    return {
        "green":  max(0, g - max(r, b)) * 50,
        "red":    max(0, r - max(g, b)) * 50,
        "yellow": max(0, min(r, g) - b) * 50,
    }


def hsv_mean(frame_rgb: np.ndarray) -> tuple[float, float, float]:
    """
    Return mean (H, S, V) of the centre 50% ROI.
    H is in OpenCV's 0-180 range; S and V are 0-255.
    """
    h, w = frame_rgb.shape[:2]
    roi  = frame_rgb[h//4:3*h//4, w//4:3*w//4]
    hsv  = cv2.cvtColor(cv2.cvtColor(roi, cv2.COLOR_RGB2BGR), cv2.COLOR_BGR2HSV)
    means = hsv.mean(axis=(0, 1))
    return float(means[0]), float(means[1]), float(means[2])


# ─── Kinetics engine (verbatim port of simulation1.py KineticsEngine) ─────────

class KineticsEngine:
    """
    Records colour transitions from cam1 and derives real-time kinetic metrics.

    Call update(colour, timestamp) on every new frame from cam1 (index 0).
    cam2 frames are stored in its own CameraCapture.history but are not fed
    into the kinetics engine — matching the original simulation1.py logic
    (only idx == 0 feeds self.kinetics.update).
    """

    def __init__(self):
        self.start_time   = time.time()
        self.transitions  = []          # list of transition event dicts
        self.prev_colour  = "none"
        self.colour_start = time.time()
        self.oscillations = 0
        self.anomaly_log  = []

    def reset(self):
        self.__init__()

    def update(self, colour: str, ts: float):
        # feed a new colour reading. Call once per cam1 frame.
        if colour == "none":
            return
        if colour != self.prev_colour:
            dur = ts - self.colour_start
            if dur > 0.4:                       # debounce flickers
                ev = {
                    "time":     round(ts - self.start_time, 2),
                    "wall":     datetime.fromtimestamp(ts).strftime("%H:%M:%S"),
                    "from_c":   self.prev_colour,
                    "to_c":     colour,
                    "duration": round(dur, 2),
                }
                self.transitions.append(ev)
                self._check_oscillation()
                self._check_anomaly(ev)
            self.prev_colour  = colour
            self.colour_start = ts

    def _check_oscillation(self):
        cols = [t["to_c"] for t in self.transitions]
        if len(cols) >= 3 and cols[-3:] == ["green", "red", "yellow"]:
            self.oscillations += 1

    def _check_anomaly(self, ev: dict):
        same = [t for t in self.transitions[:-1]
                if t["from_c"] == ev["from_c"] and t["to_c"] == ev["to_c"]]
        if len(same) >= 2:
            md = float(np.mean([t["duration"] for t in same]))
            if ev["duration"] > md * 2 or ev["duration"] < md * 0.5:
                self.anomaly_log.append(
                    f"{ev['wall']}  ⚠  {ev['from_c']}→{ev['to_c']} "
                    f"took {ev['duration']:.1f}s  (mean {md:.1f}s)"
                )

    # ── Derived metrics ───────────────────────────────────────────────────────

    def induction_period(self) -> float | None:
        real = [t for t in self.transitions if t["from_c"] != "none"]
        return round(real[0]["time"], 2) if real else None

    def phase_durations(self) -> dict:
        d = collections.defaultdict(list)
        for t in self.transitions:
            d[t["from_c"]].append(t["duration"])
        return {k: round(float(np.mean(v)), 2) for k, v in d.items() if k != "none"}

    def mean_cycle_period(self) -> float | None:
        tpc = collections.defaultdict(list)
        for t in self.transitions:
            tpc[t["to_c"]].append(t["time"])
        periods = []
        for times in tpc.values():
            if len(times) >= 2:
                periods.extend(np.diff(times).tolist())
        return round(float(np.mean(periods)), 2) if periods else None

    def concentration_proxy(self, history) -> tuple[list, list]:
        """
        Returns (times, norm_hues) for the concentration proxy graph.
        norm_hue = H / 180  (maps OpenCV hue 0-180 → 0-1 scale).
        history is a deque/list of (ts, H, S, V, colour).
        """
        if not history:
            return [], []
        t0 = self.start_time
        return (
            [p[0] - t0 for p in history],
            [p[1] / 180.0 for p in history],
        )

    def reaction_rate(self, history, window: int = 8) -> tuple[list, list]:
        """
        Returns (times, rates) for the reaction rate proxy graph.
        rate = |ΔH| / Δt over a rolling window of `window` frames.
        Optionally smoothed with a Savitzky-Golay filter if scipy is present.
        """
        if len(history) < window + 1:
            return [], []
        t0    = self.start_time
        times = [p[0] - t0 for p in history]
        hues  = np.array([p[1] for p in history], dtype=float)
        if SCIPY and len(hues) > 11:
            hues = savgol_filter(hues, min(11, len(hues) // 2 * 2 - 1), 3)
        rates, rt = [], []
        for i in range(window, len(hues)):
            dt = times[i] - times[i - window]
            rates.append(abs(hues[i] - hues[i - window]) / dt if dt > 0 else 0)
            rt.append(times[i])
        return rt, rates

    def to_dict(self) -> dict:
        return {
            "run_start":       datetime.fromtimestamp(self.start_time).isoformat(),
            "elapsed_s":       round(time.time() - self.start_time, 1),
            "oscillations":    self.oscillations,
            "induction_s":     self.induction_period(),
            "mean_cycle_s":    self.mean_cycle_period(),
            "phase_durations": self.phase_durations(),
            "transitions":     self.transitions,
            "anomalies":       self.anomaly_log,
        }


# ─── Graph data bundle ────────────────────────────────────────────────────────

def get_graph_data(history: collections.deque, kinetics: KineticsEngine) -> dict:
    """
    Returns everything the dashboard graphs need in a single dict.

    Call this on each UI refresh tick (e.g. every 100 ms).

    Parameters
    ----------
    history  : Deque of kinetics data points from the active camera.
    kinetics : KineticsEngine being fed from cam1

    Returns
    -------
    {
      "conc_times":   list[float]   # elapsed seconds
      "conc_hues":    list[float]   # normalised hue 0-1
      "conc_colours": list[str]     # per-point colour label for segment colouring
      "rate_times":   list[float]
      "rate_values":  list[float]   # |dH/dt|
      "transitions":  list[dict]    # {time, wall, from_c, to_c, duration}
      "metrics": {
          "induction_s":     float | None
          "mean_cycle_s":    float | None
          "oscillations":    int
          "phase_durations": dict   # {green, red, yellow} mean seconds
          "elapsed_s":       float
          "transition_count": int
      }
    }
    """
    hist = list(history)

    conc_times, conc_hues = kinetics.concentration_proxy(hist)
    rate_times, rate_vals = kinetics.reaction_rate(hist)
    conc_colours = [p[4] for p in hist]   # colour label per history point

    pd = kinetics.phase_durations()

    return {
        "conc_times":   conc_times,
        "conc_hues":    conc_hues,
        "conc_colours": conc_colours,
        "rate_times":   rate_times,
        "rate_values":  rate_vals,
        "transitions":  kinetics.transitions,
        "metrics": {
            "induction_s":      kinetics.induction_period(),
            "mean_cycle_s":     kinetics.mean_cycle_period(),
            "oscillations":     kinetics.oscillations,
            "phase_durations":  pd,
            "elapsed_s":        round(time.time() - kinetics.start_time, 1),
            "transition_count": len(kinetics.transitions),
        },
    }


# ─── Wiring: connect live cameras to the kinetics engine ─────────────────────

class LiveKineticsRunner:
    """
    Drop-in replacement for the camera + kinetics wiring inside
    WorkflowRunner._capture_loop / WorkflowRunner.__init__.

    Usage
    -----
        runner = LiveKineticsRunner(cam1_index=0, cam2_index=1)
        runner.start()

        # In your dashboard poll loop (e.g. every 100 ms):
        data = runner.get_graph_data()

        runner.stop()
    """

    def __init__(self, monitor1=None, monitor2=None):
        self.monitor1 = monitor1
        self.monitor2 = monitor2
        self.kinetics = KineticsEngine()

        self.history1 = collections.deque(maxlen=3000)
        self.history2 = collections.deque(maxlen=3000)

        self._running = False
        self._thread1 = None
        self._thread2 = None

        self.active_monitor = self.monitor1
        self.active_history = self.history1
        self.active_cam_name = "cam1"

    def switch_source(self) -> str:
        """
        Switches the active camera for kinetics analysis and resets the engine.
        Returns the name of the new active camera source ('cam1' or 'cam2').
        """
        if self.active_monitor == self.monitor1:
            self.active_monitor = self.monitor2
            self.active_history = self.history2
            self.active_cam_name = "cam2"
        else:
            self.active_monitor = self.monitor1
            self.active_history = self.history1
            self.active_cam_name = "cam1"
        self.kinetics.reset()
        print(f"[Kinetics] Switched source to {self.active_cam_name}")
        return self.active_cam_name

    def start(self):
        self._running = True
        if self.monitor1:
            self._thread1 = threading.Thread(target=self._processing_loop, args=(self.monitor1, self.history1), daemon=True)
            self._thread1.start()
        if self.monitor2:
            self._thread2 = threading.Thread(target=self._processing_loop, args=(self.monitor2, self.history2), daemon=True)
            self._thread2.start()

    def stop(self):
        self._running = False
        if self._thread1: self._thread1.join(timeout=1.0)
        if self._thread2: self._thread2.join(timeout=1.0)

    def _processing_loop(self, monitor, history):
        last_ts = 0.0
        while self._running:
            bgr = monitor.get_frame()
            if bgr is None:
                time.sleep(0.04)
                continue

            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            colour, _ = detect_colour(rgb)
            h, s, v = hsv_mean(rgb)
            ts = time.time()

            history.append((ts, h, s, v, colour))

            # Feed kinetics engine if this is the active source
            if self.active_monitor == monitor and ts != last_ts:
                self.kinetics.update(colour, ts)
                last_ts = ts

            time.sleep(0.04)

    def get_graph_data(self) -> dict:
        # returns the graph data bundle. Call from your dashboard poll loop.
        return get_graph_data(self.active_history, self.kinetics)
