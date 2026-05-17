"""
get_tcp.py
==========

A simple utility script to connect to the UR robot and print its
current Tool Center Point (TCP) pose to the console.

The pose is returned as [x, y, z, rx, ry, rz] in meters and radians.
"""

import sys

try:
    from utils.UR_Functions import URfunctions as URControl
except ImportError:
    print("Error: Could not import URfunctions. Make sure you are running this script from the 'GroupAv2' directory.")
    sys.exit(1)


# --- Configuration ---
HOST = "192.168.0.2"  # Standard IP address for the UR robot
PORT = 30003         # Port for primary client interface

def main():
    # connects to the robot, retrieves the current TCP pose, and prints it.
    print(f"Connecting to robot at {HOST}...")
    try:
        robot = URControl(ip=HOST, port=PORT)
        tcp_pose = robot.get_current_tcp()

        print("\n--- Current TCP Pose ---")
        print(f"  [x, y, z, rx, ry, rz]")
        print(f"  {tcp_pose.tolist()}")
        print("\nUnits are meters and radians.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
        print("Please ensure the robot is powered on and connected to the network.")

if __name__ == '__main__':
    main()