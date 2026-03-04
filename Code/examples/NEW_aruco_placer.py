"""
aruco_placer.py
================
ArUco-guided precise placement back into the vial rack hole.

Sits in the examples/ folder alongside main_v2.py.

How it works:
    1. Robot moves to a scan pose above the vial rack (joint move).
    2. Wrist camera detects the ArUco marker stuck inside/next to the rack hole.
    3. Marker pose is measured in camera frame, transformed to robot base frame
       using the current TCP pose from get_current_tcp().
    4. Robot does a precise movel_tcp descent into the hole and releases gripper.

Setup checklist:
    1. Run calibrate_camera.py to get CAMERA_MATRIX and DIST_COEFFS.
       Paste the printed values into this file below.
    2. Measure T_CAM_TCP — how far the camera is from the TCP (see below).
    3. Stick one ArUco marker (ID 0, DICT_4X4_50) flat in/next to the vial rack hole.
       Print at MARKER_SIZE_M physical size (default 40x40mm).
    4. Teach PositionList.scanPoseVialRack — hover above rack, camera pointing
       down at the marker, ~15-20cm above it.
    5. Generate the marker PNG:
           python -c "from aruco_placer import generate_rack_marker; generate_rack_marker()"
"""

import cv2
import numpy as np
import time
import os

# ──────────────────────────────────────────────────────────────────────────────
# CAMERA INTRINSICS  ← paste output from calibrate_camera.py here
# ──────────────────────────────────────────────────────────────────────────────
CAMERA_MATRIX = np.array([
    [640.0,   0.0, 320.0],
    [  0.0, 640.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float64)

DIST_COEFFS = np.zeros((1, 5), dtype=np.float64)

# ──────────────────────────────────────────────────────────────────────────────
# CAMERA-TO-TCP OFFSET
# Measure physically with a ruler (metres).
# x = camera left(+) / right(-) of TCP centreline
# y = camera forward(+) / back(-) of TCP centreline
# z = camera below TCP (negative = below, e.g. -0.05 = 5cm below TCP)
# If camera points straight down and is centred below TCP, only z changes.
# ──────────────────────────────────────────────────────────────────────────────
T_CAM_TCP = np.array([
    [1, 0, 0,  0.00],
    [0, 1, 0,  0.00],
    [0, 0, 1, -0.05],   # ← update with your measured camera-to-TCP distance
    [0, 0, 0,  1.00],
], dtype=np.float64)

# ──────────────────────────────────────────────────────────────────────────────
# MARKER CONFIG
# ──────────────────────────────────────────────────────────────────────────────
ARUCO_DICT      = cv2.aruco.DICT_4X4_50
RACK_MARKER_ID  = 0              # ID of marker stuck in/next to vial rack hole
MARKER_SIZE_M   = 0.04           # physical side length of printed marker (metres)
POSE_AVG_FRAMES = 10             # frames to average for stable pose estimate

# Fine-tune offsets applied on top of detected marker centre (metres).
# If the marker can't sit inside the hole and is placed beside it,
# adjust these so the robot aims at the hole centre not the marker centre.
PLACE_OFFSET_X = 0.0
PLACE_OFFSET_Y = 0.0
PLACE_OFFSET_Z = 0.0

# How far above the rack to hover before descending
APPROACH_HEIGHT = 0.08  # metres


# ──────────────────────────────────────────────────────────────────────────────
# MARKER GENERATION
# ──────────────────────────────────────────────────────────────────────────────
def generate_rack_marker(output_dir="."):
    """Save the rack marker PNG. Print at MARKER_SIZE_M physical size (40x40mm)."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    img = cv2.aruco.generateImageMarker(aruco_dict, RACK_MARKER_ID, 300)
    path = os.path.join(output_dir, f"rack_marker_{RACK_MARKER_ID}.png")
    cv2.imwrite(path, img)
    print(f"Saved {path}  —  print at {int(MARKER_SIZE_M*1000)}x{int(MARKER_SIZE_M*1000)}mm.")


# ──────────────────────────────────────────────────────────────────────────────
# COORDINATE HELPERS
# ──────────────────────────────────────────────────────────────────────────────
def _rvec_tvec_to_matrix(rvec, tvec):
    """Convert OpenCV rvec/tvec to 4x4 homogeneous transform."""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = tvec.flatten()
    return T


def _tcp_pose_to_matrix(tcp_pose):
    """
    Convert UR TCP pose [x, y, z, rx, ry, rz] to 4x4 homogeneous transform.
    rx/ry/rz are axis-angle (rotation vector) as returned by get_current_tcp().
    """
    x, y, z, rx, ry, rz = tcp_pose
    R, _ = cv2.Rodrigues(np.array([rx, ry, rz]))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [x, y, z]
    return T


def marker_position_in_base_frame(rvec, tvec, tcp_pose):
    """
    Transform marker centre from camera frame → robot base frame.

    Args:
        rvec     : (3,) rotation vector from estimatePoseSingleMarkers
        tvec     : (3,) translation vector from estimatePoseSingleMarkers
        tcp_pose : [x,y,z,rx,ry,rz] from robot.get_current_tcp()

    Returns:
        position : np.array [x, y, z] in robot base frame (metres)
        rv       : np.array [rx, ry, rz] orientation in base frame
    """
    T_marker_cam = _rvec_tvec_to_matrix(rvec, tvec)
    T_tcp_base   = _tcp_pose_to_matrix(tcp_pose)

    # Marker in base frame = T_tcp_base @ T_cam_tcp @ T_marker_cam
    T_marker_base = T_tcp_base @ T_CAM_TCP @ T_marker_cam

    position = T_marker_base[:3, 3]
    R_base   = T_marker_base[:3, :3]
    rv, _    = cv2.Rodrigues(R_base)

    return position, rv.flatten()


# ──────────────────────────────────────────────────────────────────────────────
# ARUCO DETECTOR
# ──────────────────────────────────────────────────────────────────────────────
class ArucoDetector:
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, params)

    def detect_single(self, frame, target_id):
        """
        Detect target_id in frame.
        Returns (rvec, tvec) or None if not found.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        ids_flat = ids.flatten().tolist()
        if target_id not in ids_flat:
            return None
        idx = ids_flat.index(target_id)
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            [corners[idx]], MARKER_SIZE_M, CAMERA_MATRIX, DIST_COEFFS
        )
        return rvec[0][0], tvec[0][0]

    def averaged_pose(self, cap, target_id, n_frames=POSE_AVG_FRAMES, timeout=8.0):
        """
        Capture n_frames containing target_id and return averaged rvec/tvec.
        Raises RuntimeError if marker not found within timeout seconds.
        """
        tvecs, rvecs = [], []
        start = time.time()
        while len(tvecs) < n_frames:
            if time.time() - start > timeout:
                raise RuntimeError(
                    f"[ArUco] Marker ID {target_id} not detected within {timeout}s.\n"
                    "Check: marker is visible from scan pose, printed correctly, "
                    "and camera index is correct."
                )
            ret, frame = cap.read()
            if not ret:
                continue
            result = self.detect_single(frame, target_id)
            if result is not None:
                rvec, tvec = result
                rvecs.append(rvec)
                tvecs.append(tvec)

        avg_tvec = np.mean(tvecs, axis=0)
        avg_rvec = np.mean(rvecs, axis=0)
        print(f"[ArUco] Marker {target_id} detected. "
              f"Camera-frame tvec (m): "
              f"x={avg_tvec[0]:.4f}  y={avg_tvec[1]:.4f}  z={avg_tvec[2]:.4f}")
        return avg_rvec, avg_tvec


# ──────────────────────────────────────────────────────────────────────────────
# MAIN PLACEMENT CLASS
# ──────────────────────────────────────────────────────────────────────────────
class ArucoPlacementSystem:
    """
    Guides precise vial placement back into the rack hole using ArUco detection.

    Usage in main_v2.py:
        aruco = ArucoPlacementSystem(robot, gripper)
        aruco.place_in_rack(PositionList.scanPoseVialRack)
    """

    def __init__(self, robot, gripper, camera_index=0):
        """
        robot        : URfunctions instance
        gripper      : RobotiqGripper instance
        camera_index : wrist camera index (0 = first/only camera on the robot)
        """
        self.robot    = robot
        self.gripper  = gripper
        self.cam_idx  = camera_index
        self.detector = ArucoDetector()

    def place_in_rack(self, scan_joint_state, place_vel=0.05, place_acc=0.2):
        """
        Full ArUco-guided placement sequence.
        Robot should be holding the vial (gripper closed) when this is called.

        Args:
            scan_joint_state : PositionList.scanPoseVialRack
            place_vel        : velocity for final descent — keep slow for precision
            place_acc        : acceleration for final descent
        """
        # ── 1. Move to scan pose above rack ─────────────────────────────────
        print("[ArUco] Moving to scan pose above rack...")
        self.robot.move_joint_list(scan_joint_state, 0.5, 0.5, 0.02)
        time.sleep(0.4)   # let arm settle before capturing

        # ── 2. Open camera, detect and average marker pose ───────────────────
        cap = cv2.VideoCapture(self.cam_idx)
        if not cap.isOpened():
            raise RuntimeError(
                f"[ArUco] Cannot open camera index {self.cam_idx}. "
                "Check it is connected and not in use."
            )
        time.sleep(0.5)   # camera warm-up frames

        try:
            rvec, tvec = self.detector.averaged_pose(cap, RACK_MARKER_ID)
        finally:
            cap.release()

        # ── 3. Transform marker position to robot base frame ────────────────
        tcp_pose = self.robot.get_current_tcp()
        marker_pos, _ = marker_position_in_base_frame(rvec, tvec, tcp_pose)

        # Apply fine-tune offsets
        marker_pos[0] += PLACE_OFFSET_X
        marker_pos[1] += PLACE_OFFSET_Y
        marker_pos[2] += PLACE_OFFSET_Z

        # ── 4. Keep current TCP orientation (gripper stays aligned) ─────────
        place_rv = tcp_pose[3:6]

        # ── 5. Approach pose: directly above the hole ────────────────────────
        approach_pose = np.array([
            marker_pos[0],
            marker_pos[1],
            marker_pos[2] + APPROACH_HEIGHT,
            place_rv[0], place_rv[1], place_rv[2]
        ])

        # ── 6. Place pose: at marker height (top of hole) ───────────────────
        place_pose = np.array([
            marker_pos[0],
            marker_pos[1],
            marker_pos[2],
            place_rv[0], place_rv[1], place_rv[2]
        ])

        print(f"[ArUco] Approach: "
              f"x={approach_pose[0]:.4f}  y={approach_pose[1]:.4f}  z={approach_pose[2]:.4f}")
        print(f"[ArUco] Place:    "
              f"x={place_pose[0]:.4f}  y={place_pose[1]:.4f}  z={place_pose[2]:.4f}")

        # ── 7. Move to approach height ───────────────────────────────────────
        print("[ArUco] Approaching above rack hole...")
        self.robot.movel_tcp(approach_pose, vel=0.2, acc=0.3)

        # ── 8. Slow descent into hole ────────────────────────────────────────
        print("[ArUco] Descending into hole (slow)...")
        self.robot.movel_tcp(place_pose, vel=place_vel, acc=place_acc)

        # ── 9. Release ───────────────────────────────────────────────────────
        self.gripper.move(0, 125, 125)
        print("[ArUco] Gripper opened — vial placed.")
        time.sleep(0.5)

        # ── 10. Retreat ──────────────────────────────────────────────────────
        print("[ArUco] Retreating...")
        self.robot.movel_tcp(approach_pose, vel=0.2, acc=0.3)
        print("[ArUco] Placement complete.")


