"""
main.py  —  Master script with colour-triggered robot control
=============================================================
Full workflow:
  1.  Pick vial from rack
  2.  Move to filming position (robot holds vial throughout)
  3.  Fixed camera watches for GREEN → RED transition
  4.  Fixed camera watches for RED → YELLOW transition
  5.  On YELLOW: shake, return to filming position
  6.  Fixed camera watches for YELLOW → RED transition
  7.  On RED: shake, return to filming position
  8.  Fixed camera watches for RED → GREEN transition
  9.  ArUco-guided placement into target slot
  10. Return home
"""

import os, sys, math
current_dir = os.path.dirname(os.path.abspath(__file__))

from utils.UR_Functions import URfunctions as URControl
from positions_list import PositionList
from utils.robotiq.robotiq_gripper import RobotiqGripper
from NEW_shake_module import ShakeModule
from NEW_aruco_placer import ArucoPlacementSystem
from NEW_colour_monitor import ColourMonitor, ColourState, ColourTimeoutError

HOST = "192.168.0.2"
PORT = 30003

# ── Which target slot to use today ──────────────────────────────────────────
# ── Colour monitor settings ──────────────────────────────────────────────────
COLOUR_CAMERA_INDEX = 1
TIMEOUT_MINS = 30.0            # applied to every colour transition wait

# ── Shake settings ───────────────────────────────────────────────────────────
SHAKE_REPETITIONS = 2
SHAKE_ANGLE_DEG   = 120


def shake_and_return(robot, shaker):
    """Move to shake position, shake, return to filming position."""
    robot.move_joint_list(PositionList.shakeVialPos, 0.5, 0.5, 0.02)
    shaker.shake(
        base_joint_state=PositionList.shakeVialPos,
        repetitions=SHAKE_REPETITIONS,
        angle_deg=SHAKE_ANGLE_DEG,
        speed=1.5,
        accel=1.5,
    )
    robot.move_joint_list(PositionList.filmingPos, 0.5, 0.5, 0.02)


def main():
    # ── Setup ────────────────────────────────────────────────────────────────
    robot   = URControl(ip=HOST, port=PORT)
    gripper = RobotiqGripper()
    gripper.connect(HOST, 63352)

    aruco   = ArucoPlacementSystem(robot, gripper)
    shaker  = ShakeModule(robot)
    monitor = ColourMonitor(camera_index=COLOUR_CAMERA_INDEX, show_preview=True)

    OpenGripper(gripper)

    try:
        # ── 1. Home ──────────────────────────────────────────────────────────
        print("\n [1] Moving to home")
        robot.move_joint_list(PositionList.defaultPos, 0.5, 0.5, 0.02)

        # ── 2. Pick vial from rack ────────────────────────────────────────────
        print("\n[2] Picking vial")
        robot.move_joint_list(PositionList.aboveVialRack, 0.5, 0.5, 0.02)
        OpenGripper(gripper)
        robot.move_joint_list(PositionList.onVialRack,    0.5, 0.5, 0.02)
        CloseGripper(gripper)
        robot.move_joint_list(PositionList.aboveVialRack, 0.5, 0.5, 0.02)

        # ── 3. Move to filming position (holding vial throughout) ─────────────
        print("\n[3] Moving to filming position")
        robot.move_joint_list(PositionList.filmingPos, 0.5, 0.5, 0.02)

        # ── 4. Wait: GREEN → RED ──────────────────────────────────────────────
        print("\n[4] Waiting for GREEN → RED")
        monitor.wait_for_transition(ColourState.GREEN, ColourState.RED, timeout_mins=TIMEOUT_MINS)
        print("GREEN → RED transition confirmed")

        # ── 5. Wait: RED → YELLOW ─────────────────────────────────────────────
        print("\n[5] Waiting for RED → YELLOW")
        monitor.wait_for_transition(ColourState.RED, ColourState.YELLOW, timeout_mins=TIMEOUT_MINS)
        print("RED → YELLOW transition confirmed. Shaking...")

        # ── 6. Shake #1, return to filming position ───────────────────────────
        print("\n[6] Shake #1")
        shake_and_return(robot, shaker)
        print("Returned to filming position.")

        # ── 7. Wait: YELLOW → RED ─────────────────────────────────────────────
        # Think just need to confirm it is RED (should change during shake)
        print("\n[7] Waiting for YELLOW → RED")
        monitor.wait_for_transition(ColourState.YELLOW, ColourState.RED, timeout_mins=TIMEOUT_MINS)
        print("YELLOW → RED transition confirmed. Shaking...")

        # ── 8. Shake #2, return to filming position ───────────────────────────
        print("\n[8] Shake #2")
        shake_and_return(robot, shaker)
        print("Returned to filming position.")

        # ── 9. Wait: RED → GREEN ──────────────────────────────────────────────
        # again this should change during the shake - might get an error when looking for a colour change
        # Might just have to confirm it is green after shaking 
        print("\n[9] Waiting for RED → GREEN")
        monitor.wait_for_transition(ColourState.RED, ColourState.GREEN,
                                    timeout_mins=TIMEOUT_MINS)
        print("RED → GREEN confirmed. Placing vial...")

        # ── 10. ArUco-guided placement into target slot ───────────────────────
        print(f"\n[10] Placing vial in slot {TARGET_SLOT_MARKER_ID} (ArUco guided)")
        aruco.pick_and_place_to_target(
            target_marker_id  = TARGET_SLOT_MARKER_ID,
            scan_joint_states = PositionList.vialRackScan,
            place_height_offset = 0.0,
        )

        # ── 11. Home ──────────────────────────────────────────────────────────
        print("\n[11] Returning to home position")
        robot.move_joint_list(PositionList.defaultPos, 0.5, 0.5, 0.02)
        OpenGripper(gripper)
        print("\nReaction complete.")

    except ColourTimeoutError as e:
        print(f"\nERROR! COLOUR TIMEOUT: {e}")
        print("Reaction may have stalled. Stopping robot safely.")
        safe_stop(robot, gripper)

    except KeyboardInterrupt:
        print("\nERROR! Interrupted by user. Stopping safely.")
        safe_stop(robot, gripper)

    finally:
        monitor.release()


def safe_stop(robot, gripper):
    """Move to home and open gripper safely."""
    try:
        robot.move_joint_list(PositionList.defaultPos, 0.2, 0.2, 0.02)
        OpenGripper(gripper)
        print("Safe stop complete - robot at home, gripper open.")
    except Exception as e:
        print(f"Safe stop error: {e}")


# ── Gripper helpers ───────────────────────────────────────────────────────────
def OpenGripper(gripper):
    gripper.move(0, 125, 125)
    print("[Gripper] Opened")

def CloseGripper(gripper):
    gripper.move(140, 125, 125)
    print("[Gripper] Closed")

def degreestorad(lst):
    return [v * (math.pi / 180) for v in lst]


if __name__ == "__main__":
    main()
