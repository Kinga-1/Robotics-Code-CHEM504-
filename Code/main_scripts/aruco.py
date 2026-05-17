import cv2
import numpy as np
import math
import calibrate_camera

RET = 0.09765356247723148

MTX = np.array([[1.16087793e+03, 0.00000000e+00, 2.94905244e+02],
       [0.00000000e+00, 1.16268262e+03, 2.98788131e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

DIST = np.array([[-1.92073629e-01, 1.27657880e+01, 1.58132911e-02, -1.96409194e-02, -1.47177006e-02]])

RVECS = np.array([[-0.06816751],
          [ 0.06253047],
          [ 0.0678527 ]])

TVECS = np.array([[-3.50562303],
         [-7.54725103],
         [39.1535462 ]])

# Define the physical size of the ArUco marker in meters.
# This MUST match the size of the printed marker.
MARKER_SIZE_M = 0.04  # 40mm

def _rvec_tvec_to_matrix(rvec, tvec):
    # convert OpenCV rvec/tvec to a 4x4 homogeneous transformation matrix.
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T

# Load the image
image = cv2.imread('scan.png')

# Add a check to ensure the image was loaded correctly
if image is None:
    print("Error: Could not load image 'scan.png'.")
    print("Please make sure the file exists in the same directory as the script and is a valid image file.")
    exit()

# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()


# #Get camera matrix, dist. coeffs
# ret, mtx, dist, rvecs, tvecs = calibrate_camera.calibrate()

# print(f'''
#       ret:
#       {ret}
#       ----
#       mtx:
#       {mtx}
#       ----
#       dist:
#       {dist}
#       ----
#       rvecs:
#       {rvecs}
#       ----
#       tvecs:
#       {tvecs}
#       ----
      
#       '''

scan2 = cv2.imread("board.png")
if scan2 is not None:
    h, w = scan2.shape[:2]
    newMTX, roi = cv2.getOptimalNewCameraMatrix(MTX, DIST, (w, h), 0.5, (w, h))
    
    dst = cv2.undistort(scan2, MTX, DIST, None, newMTX)
    
    # x, y, w, h = roi # This line is not used, can be removed or commented.
    cv2.imwrite('calibresult.png', dst)
    print("Undistortion result saved to 'calibresult.png'.")
else:
    print("Warning: 'board.png' not found. Skipping undistortion demonstration.")

# Create the ArUco detector
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
# Detect the markers
corners, ids, rejected = detector.detectMarkers(gray)

# Print the detected markers
print("Detected markers:", ids)
if ids is not None:
    # Estimate the pose of each marker
    # The function `estimatePoseSingleMarkers` is deprecated in newer OpenCV versions.
    # We will use `cv2.solvePnP` for each marker instead.
    s = MARKER_SIZE_M / 2.0
    # 3D points of the marker corners in its own coordinate system
    object_points = np.array([
        [-s,  s, 0], # Top-left
        [ s,  s, 0], # Top-right
        [ s, -s, 0], # Bottom-right
        [-s, -s, 0]  # Bottom-left
    ], dtype=np.float32)

    rvecs_list = []
    tvecs_list = []
    for marker_corners in corners:
        # Use solvePnP for each marker to get its pose
        retval, rvec, tvec = cv2.solvePnP(object_points, marker_corners, MTX, DIST)
        rvecs_list.append(rvec)
        tvecs_list.append(tvec)
    
    rvecs, tvecs = np.array(rvecs_list), np.array(tvecs_list)

    # Define the 3D coordinates of a marker's corners in its own local coordinate system.
    # The center of the marker is the origin (0,0,0).
    s = MARKER_SIZE_M / 2
    marker_corners_3d_local = np.array([
        [-s,  s, 0, 1],  # Top-left (homogeneous coordinate)
        [ s,  s, 0, 1],  # Top-right
        [ s, -s, 0, 1],  # Bottom-right
        [-s, -s, 0, 1]   # Bottom-left
    ], dtype=np.float32).T # Transpose to shape (4, 4) for matrix multiplication

    # The 'corners' variable is a list of numpy arrays.
    # Each array contains the (x, y) pixel coordinates of the four corners for one marker.
    # The order is top-left, top-right, bottom-right, and bottom-left.
    for i, marker_id in enumerate(ids):
        print(f"\nMarker ID: {marker_id[0]}")
        marker_corners = corners[i][0]  # The corners are wrapped in an extra array
        print(f"  2D Corner Coordinates (in pixels):")
        print(f"  - Top-left:     ({marker_corners[0][0]:.0f}, {marker_corners[0][1]:.0f})")
        print(f"  - Top-right:    ({marker_corners[1][0]:.0f}, {marker_corners[1][1]:.0f})")
        print(f"  - Bottom-right: ({marker_corners[2][0]:.0f}, {marker_corners[2][1]:.0f})")
        print(f"  - Bottom-left:  ({marker_corners[3][0]:.0f}, {marker_corners[3][1]:.0f})")

        # Get the pose for this specific marker
        rvec = rvecs[i]
        tvec = tvecs[i]

        # Convert rvec/tvec to a 4x4 transformation matrix (T_marker_cam)
        # This matrix transforms points from the marker's frame to the camera's frame.
        T_marker_cam = _rvec_tvec_to_matrix(rvec, tvec)

        # Transform the 3D marker corners from the marker's local frame to the camera's frame
        corners_in_cam_frame = T_marker_cam @ marker_corners_3d_local

        print(f"\n  3D Corner Coordinates (in camera frame, meters):")
        print(f"  - Top-left:     (x={corners_in_cam_frame[0,0]:.4f}, y={corners_in_cam_frame[1,0]:.4f}, z={corners_in_cam_frame[2,0]:.4f})")
        print(f"  - Top-right:    (x={corners_in_cam_frame[0,1]:.4f}, y={corners_in_cam_frame[1,1]:.4f}, z={corners_in_cam_frame[2,1]:.4f})")
        print(f"  - Bottom-right: (x={corners_in_cam_frame[0,2]:.4f}, y={corners_in_cam_frame[1,2]:.4f}, z={corners_in_cam_frame[2,2]:.4f})")
        print(f"  - Bottom-left:  (x={corners_in_cam_frame[0,3]:.4f}, y={corners_in_cam_frame[1,3]:.4f}, z={corners_in_cam_frame[2,3]:.4f})")

    cv2.aruco.drawDetectedMarkers(image, corners, ids)
    cv2.imshow('Detected Markers', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

print("\n---")
print("To convert from Camera Frame to Robot Base Frame, you would then apply:")
print("1. The Camera-to-TCP transformation (physical offset of camera on the tool).")
print("2. The TCP-to-Base transformation (current pose of the robot's tool).")
print("See the function 'marker_position_in_base_frame' in 'aruco_placer.py' for a full example.")
