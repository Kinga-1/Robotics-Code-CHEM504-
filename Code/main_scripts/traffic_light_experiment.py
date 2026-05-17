import math
import numpy as np
import cv2
import time

from positions_list import PositionList
from tcp_positions import VIAL_RACK_Z, STIRRING_PLATE_Z, FILMING_POSE, APPROACH_DISTANCE
from gripper_helpers import OpenGripper, CloseGripper
from colour_monitor import ColourState

'''
NOTES

Measurements in TCP space, in mm | [x, y, z, rx, ry, rz]
Corner positions:
bottom right:  [283, -248, -352, 0, 3.14, 0]
 bottom left:  [324, -651, -352, 0, 3.14, 0]
   top right:  [-209, -296, -352, 0, 3.14, 0]
    top left:  [-168, -701, -352, 0, 3.14, 0]

'''



# ── Colour monitor settings ──────────────────────────────────────────────────
COLOUR_CAMERA_INDEX = 0
TIMEOUT_MINS = 30.0            # applied to every colour transition wait

WRIST_CAMERA_INDEX = 1
REACTION_CAMERA_INDEX = 0

# ── Shake settings ───────────────────────────────────────────────────────────
SHAKE_REPETITIONS = 2
SHAKE_ANGLE_DEG   = 120

TARGET_SLOT_MARKER_ID = 1

# -- Plate settings --------------------
STIRRING_SPEED = "1500"
STIRRING_TIME = 20

def main(display, robot, gripper, ih, filmMonitor, plateMonitor, aruco, chosen_mode, on_complete=None):
    """
    Main experiment function.
    
    Args:
        display: UI display object.
        robot: URControl object.
        gripper: RobotiqGripper object.
        ih: IKAHandler object for stirrer.
        filmMonitor: ColourMonitor for filming camera.
        plateMonitor: ColourMonitor for plate camera.
        aruco: ArucoPlacementSystem object.
        chosen_mode (str): 'A' for Aruco, 'C' for Coordinate.
        on_complete (callable, optional): Callback to run when the experiment finishes successfully.
    """
    frame_no = 0

    # --- Mode Selection ---
    use_aruco = (chosen_mode == 'A')

    # --- Define all target poses based on mode ---
    if use_aruco:
        display.log_event("ARUCO MODE: Locating objects with vision...")
        try:
            # Find vial hole. This moves robot to scan pose.
            vial_hole_coords = aruco.find_vial_rack_holes(PositionList.scanAreaPos)
            if not vial_hole_coords: raise RuntimeError("No vial holes found.")
            target_hole_xyz = vial_hole_coords[0]
            display.log_event(f"[ARUCO] Found vial hole at: {np.round(target_hole_xyz, 4)}")

            # Find stirring plate. This also moves robot to scan pose.
            stir_plate_coords = aruco.find_stirring_plate(PositionList.scanAreaPos)
            if not stir_plate_coords: raise RuntimeError("Stirring plate not found.")
            plate_center_xyz = stir_plate_coords[0]
            display.log_event(f"[ARUCO] Found stirring plate at: {np.round(plate_center_xyz, 4)}")

            # Define TCP poses from vision results
            base_orientation = FILMING_POSE[3:]

            # Vial Rack Poses
            vial_rack_approach_pose = [*target_hole_xyz[:2], target_hole_xyz[2] + APPROACH_DISTANCE, *base_orientation]
            vial_rack_on_pose = [*target_hole_xyz, *base_orientation]
            vial_rack_retreat_pose = [*target_hole_xyz[:2], target_hole_xyz[2] + 0.15, *base_orientation] # 15cm above

            # Stirring Plate Poses
            stirrer_approach_pose = [*plate_center_xyz[:2], plate_center_xyz[2] + APPROACH_DISTANCE, *base_orientation]
            stirrer_on_pose = [*plate_center_xyz, *base_orientation]
            stirrer_retreat_pose = [*plate_center_xyz[:2], plate_center_xyz[2] + 0.15, *base_orientation] # 15cm above

            # Use a TCP-based move for filming as well for consistency
            filming_pose = FILMING_POSE

        except Exception as e:
            display.log_event(f"FATAL: Vision setup failed: {e}. Aborting.")
            return
    else:
        display.log_event("COORDINATE MODE: Using hard-coded positions.")

    ih.setStirringSpeed(STIRRING_SPEED)

    # home position
    display.log_event("Moving to home position")
    OpenGripper(gripper)
    robot.move_joint_list(PositionList.defaultPos, 0.5, 0.5, 0.02)

    # --- Pick vial from rack ---
    display.log_event("[EXPERIMENT] Picking vial from rack")
    if use_aruco:
        robot.movel_tcp(vial_rack_approach_pose, 0.2, 0.2)
        robot.movel_tcp(vial_rack_on_pose, 0.1, 0.1)
        CloseGripper(gripper)
        robot.movel_tcp(vial_rack_approach_pose, 0.2, 0.2)
    else:
        robot.move_joint_list(PositionList.aboveVialRack, 0.5, 0.5, 0.02)
        robot.move_joint_list(PositionList.onVialRack, 0.5, 0.5, 0.02)
        CloseGripper(gripper)
        robot.move_joint_list(PositionList.aboveVialRack, 0.5, 0.5, 0.02)

    # --- Move to filming position ---
    display.log_event("[EXPERIMENT] Moving to filming position")
    if use_aruco:
        robot.movel_tcp(filming_pose, 0.5, 0.5)
    else:
        robot.move_joint_list(PositionList.filmingPos, 0.5, 0.5, 0.02)
    
    display.log_event("[EXPERIMENT] Waiting for GREEN -> RED")
    filmMonitor.wait_for_transition(ColourState.GREEN, ColourState.RED)
    
    display.log_event("[EXPERIMENT]Waiting for RED -> YELLOW")
    filmMonitor.wait_for_transition(ColourState.RED, ColourState.YELLOW)
    
    frameCapture(filmMonitor.cap, frame_no)
    frame_no += 1        
    
    # --- Move to stirring plate (Cycle 1) ---
    display.log_event("[ROBOT] Moving vial to stirring plate")
    if use_aruco:
        robot.movel_tcp(stirrer_retreat_pose, 0.5, 0.5)
        robot.movel_tcp(stirrer_approach_pose, 0.2, 0.2)
        robot.movel_tcp(stirrer_on_pose, 0.1, 0.1)
    else:
        robot.move_joint_list(PositionList.aboveStirringPlate, 0.5, 0.5, 0.02)
        robot.move_joint_list(PositionList.hoverStirringPlate, 0.5, 0.5, 0.02)

    display.log_event("[CAMERA] Switching graph source to Plate Camera")
    display.programmatic_switch_kinetics_source()
    
    ih.startStirring()
    
    display.log_event("[EXPERIMENT] Waiting for YELLOW -> RED")
    plateMonitor.wait_for_transition(ColourState.YELLOW, ColourState.RED)
            
    ih.stopStirring()
    
    # --- Retreat from stirring plate and move to filming position ---
    display.log_event("[EXPERIMENT] Retreating from stirring plate")
    if use_aruco:
        robot.movel_tcp(stirrer_approach_pose, 0.2, 0.2)
        display.log_event("[EXPERIMENT] Moving to filming position")
        robot.movel_tcp(filming_pose, 0.5, 0.5)
    else:
        robot.move_joint_list(PositionList.aboveStirringPlate, 0.5, 0.5, 0.02)
        display.log_event("Moving to filming position")
        robot.move_joint_list(PositionList.filmingPos, 0.5, 0.5, 0.02)

    display.log_event("[CAMERA] Switching graph source to Filming Camera")
    display.programmatic_switch_kinetics_source()
    
    time.sleep(2)
    
    frameCapture(filmMonitor.cap, frame_no)
    frame_no += 1
    
    # --- Move to stirring plate (Cycle 2) ---
    display.log_event("[EXPERIMENT] Moving vial to stirring plate")
    if use_aruco:
        robot.movel_tcp(stirrer_retreat_pose, 0.5, 0.5)
        robot.movel_tcp(stirrer_approach_pose, 0.2, 0.2)
        robot.movel_tcp(stirrer_on_pose, 0.1, 0.1)
    else:
        robot.move_joint_list(PositionList.aboveStirringPlate, 0.5, 0.5, 0.02)
        robot.move_joint_list(PositionList.hoverStirringPlate, 0.5, 0.5, 0.02)

    ih.startStirring()
    
    display.log_event("[EXPERIMENT] Waiting for RED -> GREEN")
    plateMonitor.wait_for_transition(ColourState.RED, ColourState.GREEN)
            
    ih.stopStirring()
    
    # --- Retreat from stirring plate and move to filming position ---
    display.log_event("[EXPERIMENT] Retreating from stirring plate")
    if use_aruco:
        robot.movel_tcp(stirrer_approach_pose, 0.2, 0.2)
        display.log_event("[EXPERIMENT] Moving to filming position")
        robot.movel_tcp(filming_pose, 0.5, 0.5)
    else:
        robot.move_joint_list(PositionList.aboveStirringPlate, 0.5, 0.5, 0.02)
        display.log_event("[EXPERIMENT] Moving to filming position")
        robot.move_joint_list(PositionList.filmingPos, 0.5, 0.5, 0.02)

    time.sleep(2)
    
    frameCapture(filmMonitor.cap, frame_no)
    frame_no += 1
    
    # --- Return vial to rack ---
    display.log_event("[EXPERIMENT] Returning vial to rack")
    if use_aruco:
        robot.movel_tcp(vial_rack_retreat_pose, 0.5, 0.5)
        robot.movel_tcp(vial_rack_approach_pose, 0.2, 0.2)
        robot.movel_tcp(vial_rack_on_pose, 0.1, 0.1)
        OpenGripper(gripper)
        robot.movel_tcp(vial_rack_approach_pose, 0.2, 0.2)
    else:
        robot.move_joint_list(PositionList.aboveVialRack, 0.5, 0.5, 0.02)
        robot.move_joint_list(PositionList.onVialRack, 0.5, 0.5, 0.02)
        OpenGripper(gripper)
        robot.move_joint_list(PositionList.aboveVialRack, 0.5, 0.5, 0.02)

    # --- Move home ---
    display.log_event("[EXPERIMENT] Returning to home position")
    robot.move_joint_list(PositionList.defaultPos, 0.5, 0.5, 0.02)

    if on_complete:
        on_complete()

    display.log_event("Experiment complete")


def degreestorad(lst):
    # return [v * (math.pi / 180) for v in lst]
    return [v * (math.pi / 180) for v in lst]

def frameCapture(c, n):
    # if c is None or not c.isOpened():
    if c is None or not c.isOpened():
        print(f"Warning: frameCapture skipped because camera is not opened for frame {n}.")
        return

    for i in range(5):
        ret, frame = c.read()
    
    if not ret or frame is None or frame.size == 0:
        print(f"Warning: Failed to capture frame {n}. ret={ret}, frame={None if frame is None else getattr(frame, 'shape', 'unknown')}")
        return

    cv2.imshow("capture", frame)
    img_name = "traffic_light_frame_{}.png".format(n)
    cv2.imwrite(img_name, frame)
    print("Saved image")
