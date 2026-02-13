import os
import time
import argparse
import math
from robotiq.robotiq_gripper import RobotiqGripper
import sys
# Add the directory containing robotiq_preamble.py to the Python search path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, 'robotiq'))

from utils.UR_Functions import URfunctions as URControl
def main():
        robot = URControl(ip="192.168.0.2", port=30003)
        gripper=RobotiqGripper()
        gripper.connect("192.168.0.2", 63352)
        joint_state = [1.232142448425293, -1.232219324713089, 1.501033131276266, -1.7906872234740199, -1.559796158467428, 0.02270333841443062]
        robot.move_joint_list(joint_state, 0.25, 0.5, 0.02)
        gripper.move(225,125,125)
        joint_state = [1.1481959819793701, -1.0524450105479737, 1.6058357397662562, -2.1035672626891078, -1.5133102575885218, 1.1738598346710205]
        robot.move_joint_list(joint_state, 0.25, 0.5, 0.02)

if __name__ == '__main__':
     main()