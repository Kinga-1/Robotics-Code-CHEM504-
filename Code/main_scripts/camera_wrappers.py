import requests
import numpy as np
import cv2

class RobotiqCamera:
    """
    Drop-in replacement for cv2.VideoCapture that pulls frames
    from the Robotiq Wrist Camera URCap HTTP endpoint.

    Usage:
        cap = RobotiqCamera("192.168.0.2")
        ret, frame = cap.read()   # same interface as VideoCapture
        cap.release()             # no-op, but keeps the interface consistent
    """

    URL_TEMPLATE = "http://{ip}:4242/current.jpg?type=color"

    def __init__(self, robot_ip, timeout=3.0):
        self.url     = self.URL_TEMPLATE.format(ip=robot_ip)
        self.timeout = timeout
        # Verify connection immediately so failures are caught early
        ret, _ = self.read()
        if not ret:
            raise RuntimeError(
                f"[RobotiqCamera] Cannot reach camera at {self.url}. "
                "Check the robot IP, that the URCap is running, "
                "and that port 4242 is reachable from this machine."
            )
        print(f"[RobotiqCamera] Connected: {self.url}")

    def read(self):
        # fetch one frame. Returns (True, frame_bgr) or (False, None).
        try:
            resp = requests.get(self.url, timeout=self.timeout)
            resp.raise_for_status()
            jpg   = np.frombuffer(resp.content, dtype=np.uint8)
            frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
            return (frame is not None), frame
        except Exception as e:
            print(f"[RobotiqCamera] read() failed: {e}")
            return False, None

    def release(self):
        pass   # nothing to release for HTTP