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

'''
NOTES

Measurements in TCP space, in mm | [x, y, z, rx, ry, rz]
Corner positions:
bottom right:  [283, -248, -352, 0, 3.14, 0]
 bottom left:  [324, -651, -352, 0, 3.14, 0]
   top right:  [-209, -296, -352, 0, 3.14, 0]
    top left:  [-168, -701, -352, 0, 3.14, 0]

'''

import cv2
import numpy as np
import time
import os

from camera_wrappers import RobotiqCamera

# OBJECT HEIGHT VARIABLES

STIRRING_PLATE_HEIGHT_M = 0.02  # metres. Height of the stirring plate surface from the robot base.
RACK_HOLE_DEPTH_M = 0.05          # metres. Depth of the vial

# ──────────────────────────────────────────────────────────────────────────────
# CAMERA INTRINSICS  ← paste output from calibrate_camera.py here
# ──────────────────────────────────────────────────────────────────────────────
CAMERA_MATRIX = np.array([
    [1.16087793e+03, 0.00000000e+00, 2.94905244e+02],
    [0.00000000e+00, 1.16268262e+03, 2.98788131e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float64)

DIST_COEFFS = np.array(
    [[-1.92073629e-01, 1.27657880e+01, 1.58132911e-02, -1.96409194e-02, -1.47177006e+02]]
, dtype=np.float64)

# ──────────────────────────────────────────────────────────────────────────────
# CAMERA-TO-TCP OFFSET
# This is the pose of the camera's coordinate frame relative to the TCP frame.
# These values are estimates for a Robotiq Wrist Camera mounted to a HAND-E gripper.
#
# - Robotiq Wrist Camera thickness: 46 mm
# - Robotiq Wrist Camera lens is offset from center by 28 mm.
# - Robotiq HAND-E TCP is ~136.4 mm from its mounting surface.
#
# x = 0.0 m   (assuming gripper is centered on camera body)
# y = 0.028 m (camera lens offset. This might be -0.028 depending on mounting rotation)
# z = -(0.046 + 0.1364) = -0.1824 m (camera is "above" the TCP, along the negative Z-axis of the tool)
# ──────────────────────────────────────────────────────────────────────────────
T_CAM_TCP = np.array([
    [1, 0, 0,  0.0000],  # x-offset
    [0, 1, 0,  0.0280],  # y-offset (lens offset)
    [0, 0, 1, -0.1824],  # z-offset (camera thickness + gripper length)
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
# HOUGH CIRCLE CONFIG
# ──────────────────────────────────────────────────────────────────────────────
# Parameters for cv2.HoughCircles, tuned for specific objects.
# These will likely need to be adjusted based on camera height and lighting.
HOUGH_VIAL_HOLE_PARAMS = {
    "dp": 1,
    "minDist": 20,         # pixels, min distance between detected centers
    "param1": 50,          # Upper threshold for the Canny edge detector
    "param2": 30,          # Accumulator threshold for the circle centers
    "minRadius": 5,        # pixels
    "maxRadius": 15        # pixels
}

HOUGH_STIR_PLATE_PARAMS = {
    "dp": 1.5,
    "minDist": 100,
    "param1": 100,
    "param2": 50,
    "minRadius": 40,
    "maxRadius": 80
}

# ──────────────────────────────────────────────────────────────────────────────
# WORKSPACE GEOMETRY (for 2D-to-3D projection)
# ──────────────────────────────────────────────────────────────────────────────
# If your workspace (breadboard) is level, you only need to define its height
# relative to the robot's base. Measure this once with a ruler.
# This avoids needing to do a 3-point plane calibration.
WORKSPACE_Z_HEIGHT_M = -0.352  # metres. From corner measurements.

# Map of the four ArUco markers at the corners of your workspace.
# The keys are the marker IDs.
# The values are their (x, y) coordinates in meters in the robot base frame.
# These are from the TCP measurements at the top of the file.
WORKSPACE_CORNERS = {
    # NOTE: The marker IDs (keys) must match the physical markers placed at these world coordinates.
    # The names 'top-left' etc. are from the original file comment and may not match visual layout.
    10: (-0.168, -0.701), # 'top left'
    11: (-0.209, -0.296), # 'top right'
    12: (0.283, -0.248),  # 'bottom right'
    13: (0.324, -0.651),  # 'bottom left'
}


# ──────────────────────────────────────────────────────────────────────────────
# MARKER GENERATION
# ──────────────────────────────────────────────────────────────────────────────
def generate_rack_marker(output_dir=".", id=RACK_MARKER_ID):
    # save the rack marker PNG. Print at MARKER_SIZE_M physical size (40x40mm).
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    img = cv2.aruco.generateImageMarker(aruco_dict, id, 300)
    path = os.path.join(output_dir, f"rack_marker_{id}.png")
    cv2.imwrite(path, img)
    print(f"Saved {path}  —  print at {int(MARKER_SIZE_M*1000)}x{int(MARKER_SIZE_M*1000)}mm.")


# ──────────────────────────────────────────────────────────────────────────────
# CIRCLE DETECTION
# ──────────────────────────────────────────────────────────────────────────────
def find_circles(image, dp, minDist, param1, param2, minRadius, maxRadius):
    # finds circles in an image using Hough Circle Transform.
    if image is None:
        return []
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Blur helps reduce noise and false detections
    gray = cv2.medianBlur(gray, 5)

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=dp,
        minDist=minDist,
        param1=param1,
        param2=param2,
        minRadius=minRadius,
        maxRadius=maxRadius
    )

    if circles is not None:
        return circles[0, :] # Returns as float array [[x, y, r], ...]
    return []

# ──────────────────────────────────────────────────────────────────────────────
# COORDINATE HELPERS
# ──────────────────────────────────────────────────────────────────────────────
def _rvec_tvec_to_matrix(rvec, tvec):
    # convert OpenCV rvec/tvec to 4x4 homogeneous transform.
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

        # The function `estimatePoseSingleMarkers` is deprecated in newer OpenCV versions.
        # We will use `cv2.solvePnP` instead.
        s = MARKER_SIZE_M / 2.0
        object_points = np.array([
            [-s,  s, 0], # Top-left
            [ s,  s, 0], # Top-right
            [ s, -s, 0], # Bottom-right
            [-s, -s, 0]  # Bottom-left
        ], dtype=np.float32)

        retval, rvec, tvec = cv2.solvePnP(object_points, corners[idx], CAMERA_MATRIX, DIST_COEFFS)
        return rvec.flatten(), tvec.flatten()

    def averaged_pose(self, cap, target_id, n_frames=POSE_AVG_FRAMES, timeout=8.0):
        """
        Capture n_frames containing target_id and return averaged rvec/tvec.
        Raises RuntFimeError if marker not found within timeout seconds.
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

    def __init__(self, robot, gripper):
        """
        robot        : URfunctions instance
        gripper      : RobotiqGripper instance
        """
        self.robot    = robot
        self.gripper  = gripper
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
        cap = RobotiqCamera(self.robot.ip) # Assumes robot object has .ip attribute
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

    def scan_workspace(self, scan_joint_state, show_detections=True):
        # scans the workspace once to find all circular objects (vial holes and stir plate).
        print(f"[CircleFinder] Moving to scan pose...")
        self.robot.move_joint_list(scan_joint_state, 0.5, 0.5, 0.02)
        time.sleep(1.0)  # Settle

        # Capture image
        cap = RobotiqCamera(self.robot.ip)
        time.sleep(0.5)
        ret, frame = cap.read()
        cap.release()
        if not ret:
            raise RuntimeError("[CircleFinder] Failed to capture image from wrist camera.")

        # Get homography from workspace corners
        try:
            homography_matrix = get_workspace_homography(frame, WORKSPACE_CORNERS)
        except RuntimeError as e:
            print(f"[CircleFinder] Error getting homography: {e}")
            cv2.imshow("Homography Failed Frame", frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            raise

        # Find vial holes
        vial_circles = find_circles(frame, **HOUGH_VIAL_HOLE_PARAMS)
        print(f"[CircleFinder] Detected {len(vial_circles)} vial hole(s).")
        vial_hole_coords = []
        for (u, v, r) in vial_circles:
            world_xyz = project_pixel_to_world_via_homography((u, v), homography_matrix, WORKSPACE_Z_HEIGHT_M)
            vial_hole_coords.append(world_xyz)

        # Find stirring plate
        stir_plate_circles = find_circles(frame, **HOUGH_STIR_PLATE_PARAMS)
        print(f"[CircleFinder] Detected {len(stir_plate_circles)} stirring plate(s).")
        stir_plate_coords = []
        for (u, v, r) in stir_plate_circles:
            world_xyz = project_pixel_to_world_via_homography((u, v), homography_matrix, STIRRING_PLATE_HEIGHT_M)
            stir_plate_coords.append(world_xyz)

        # (Optional) Show visualization
        if show_detections and frame is not None:
            vis_frame = frame.copy()
            for (u, v, r) in vial_circles:
                cv2.circle(vis_frame, (int(u), int(v)), int(r), (0, 255, 0), 2) # Green for vial holes
            for (u, v, r) in stir_plate_circles:
                cv2.circle(vis_frame, (int(u), int(v)), int(r), (255, 0, 0), 2) # Blue for stir plate
            cv2.imshow("Workspace Detections", vis_frame)
            print("[CircleFinder] Showing detections. Press any key in the window to continue.")
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return vial_hole_coords, stir_plate_coords


# ──────────────────────────────────────────────────────────────────────────────
# 2D-to-3D PROJECTION HELPERS (for Hough Circle, etc.)
# ──────────────────────────────────────────────────────────────────────────────

def project_pixel_to_z_plane(pixel_uv, robot_tcp_pose, z_height):
    """
    Projects a 2D pixel coordinate from the camera image onto a horizontal plane
    at a fixed z_height in the robot's base frame.

    This is a simplified version of project_pixel_to_plane for when the
    workspace is a known, level height. This avoids the need to perform a
    3-point plane calibration.

    Args:
        pixel_uv (tuple): The (u, v) or (x, y) pixel coordinate from the image.
        robot_tcp_pose (list): The robot's TCP pose [x,y,z,rx,ry,rz] at the
                               moment the image was captured.
        z_height (float): The Z-coordinate (height) of the workspace plane in
                          the robot's base frame (in metres).

    Returns:
        np.array: The [x, y, z] world coordinates of the pixel on the plane.
                  The z value will be equal to z_height.
    """
    # Define a horizontal plane at the given Z height
    plane_normal = np.array([0.0, 0.0, 1.0])
    plane_point = np.array([0.0, 0.0, z_height])

    # Reuse the general-purpose projection function with these parameters
    return project_pixel_to_plane(pixel_uv, robot_tcp_pose, plane_normal, plane_point)

def calculate_plane_from_points(p1, p2, p3):
    """
    Calculates the plane equation (normal vector and a point) from three 3D points.
    The plane is defined by the normal vector and one of the points.

    Args:
        p1, p2, p3: np.array([x, y, z]) points in the robot base frame.

    Returns:
        (normal, point): A tuple containing the normalized plane normal vector
                         and the first point (p1) which lies on the plane.
    """
    p1 = np.asarray(p1)
    p2 = np.asarray(p2)
    p3 = np.asarray(p3)
    v1 = p2 - p1
    v2 = p3 - p1
    normal = np.cross(v1, v2)
    norm_mag = np.linalg.norm(normal)
    if norm_mag == 0:
        raise ValueError("Points are collinear and do not define a plane.")
    
    # Ensure the normal points "up" relative to the base frame's Z-axis
    if normal[2] < 0:
        normal = -normal

    return normal / norm_mag, p1


def project_pixel_to_plane(pixel_uv, robot_tcp_pose, plane_normal, plane_point):
    """
    Projects a 2D pixel coordinate from the camera image onto a 3D plane in the
    robot's base frame to find its world coordinates.

    This is the core function to turn a 2D detection (like a Hough Circle center)
    into a 3D target for the robot.

    Args:
        pixel_uv (tuple): The (u, v) or (x, y) pixel coordinate from the image.
        robot_tcp_pose (list): The robot's TCP pose [x,y,z,rx,ry,rz] at the
                               moment the image was captured.
        plane_normal (np.array): The normalized normal vector of the target plane
                                 in the robot base frame.
        plane_point (np.array): A 3D point that lies on the target plane in the
                                robot base frame.

    Returns:
        np.array: The [x, y, z] world coordinates of the pixel on the plane.
    """
    u, v = pixel_uv

    # 1. Get camera intrinsic parameters from the global constants
    fx = CAMERA_MATRIX[0, 0]
    fy = CAMERA_MATRIX[1, 1]
    cx = CAMERA_MATRIX[0, 2]
    cy = CAMERA_MATRIX[1, 2]

    # 2. Create a 3D ray in the CAMERA's coordinate frame.
    # This ray originates from the camera's center (0,0,0) and passes through
    # the pixel (u,v) on the image plane.
    direction_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    direction_cam /= np.linalg.norm(direction_cam) # Normalize

    # 3. Transform the ray from the CAMERA frame to the ROBOT BASE frame.
    T_tcp_base = _tcp_pose_to_matrix(robot_tcp_pose)
    T_cam_base = T_tcp_base @ T_CAM_TCP

    camera_center_base = T_cam_base[:3, 3]
    R_cam_base = T_cam_base[:3, :3]
    direction_base = R_cam_base @ direction_cam

    # 4. Calculate the intersection of the ray with the plane.
    # The ray is P(t) = camera_center_base + t * direction_base
    # The plane is defined by plane_normal . (X - plane_point) = 0
    # We substitute P(t) for X and solve for t.
    denom = np.dot(plane_normal, direction_base)
    if abs(denom) < 1e-6:
        raise RuntimeError("Camera ray is parallel to the workspace plane.")

    numer = np.dot(plane_normal, plane_point - camera_center_base)
    t = numer / denom

    # 5. Find the 3D intersection point using the calculated t.
    intersection_point = camera_center_base + t * direction_base

    print(f"[Projection] Pixel ({u},{v}) -> World "
          f"x={intersection_point[0]:.4f} "
          f"y={intersection_point[1]:.4f} "
          f"z={intersection_point[2]:.4f}")

    return intersection_point

# ──────────────────────────────────────────────────────────────────────────────
# HOMOGRAPHY-BASED PROJECTION (Alternative to Ray-Plane Intersection)
# ──────────────────────────────────────────────────────────────────────────────

def get_workspace_homography(image, workspace_corners_map):
    """
    Calculates the homography matrix to map image pixels to world coordinates.
    This is done by finding 4+ ArUco markers with known world positions.

    Args:
        image (np.array): The image captured from a fixed perspective of the workspace.
        workspace_corners_map (dict): Maps marker IDs to their known (x, y) world coords.
                                      e.g., {10: (0.0, 0.0), 11: (0.5, 0.0), ...}

    Returns:
        np.array: The 3x3 homography matrix.

    Raises:
        RuntimeError: If fewer than 4 markers from the map are found.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        raise RuntimeError("No ArUco markers detected in the image.")

    src_points = []  # Pixel coordinates (marker centers)
    dst_points = []  # World coordinates (from map)

    for i, marker_id in enumerate(ids.flatten()):
        if marker_id in workspace_corners_map:
            # Get the pixel coordinates of the marker's corners
            marker_corners = corners[i][0]
            # Calculate the center of the marker
            center_pixel = np.mean(marker_corners, axis=0)
            
            src_points.append(center_pixel)
            dst_points.append(workspace_corners_map[marker_id])

    if len(src_points) < 4:
        raise RuntimeError(f"Found only {len(src_points)} of the required 4+ workspace markers.")

    # Convert to numpy arrays for cv2.findHomography
    src_points = np.array(src_points, dtype=np.float32)
    dst_points = np.array(dst_points, dtype=np.float32)

    homography_matrix, _ = cv2.findHomography(src_points, dst_points)
    print(f"[Homography] Calculated from {len(src_points)} markers.")
    return homography_matrix


def project_pixel_to_world_via_homography(pixel_uv, homography_matrix, z_height):
    """
    Projects a 2D pixel coordinate to 3D world coordinates using a homography matrix.

    Args:
        pixel_uv (tuple): The (u, v) or (x, y) pixel coordinate from the image.
        homography_matrix (np.array): The 3x3 homography matrix from get_workspace_homography.
        z_height (float): The known Z-height of the workspace plane.

    Returns:
        np.array: The [x, y, z] world coordinates of the pixel on the plane.
    """
    pixel_uv_np = np.array([[pixel_uv]], dtype=np.float32) # Needs to be in a 3D array for perspectiveTransform

    # Apply the homography to get the world (x, y)
    world_xy = cv2.perspectiveTransform(pixel_uv_np, homography_matrix)

    x = world_xy[0][0][0]
    y = world_xy[0][0][1]

    return np.array([x, y, z_height])