
from PIL import ImageTk, Image
import numpy as np
import math
import os
import json
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, 'robotiq'))
from utils.UR_Functions import URfunctions as URControl
from robotiq.robotiq_gripper import RobotiqGripper
def main():
    with open('pickplaceVial_JH.json', 'r') as file:
        data = json.load(file)
    robot = URControl(ip="192.168.0.2", port=30003)
    gripper=RobotiqGripper()
    gripper.connect("192.168.0.2", 63352)
    gripper.move(0,125,125)
    for key, value in data.items():
        if key.startswith('move'):
            robot.move_joint_list(value,0.25,0.25,0.02)
        elif key.startswith('gripper'):
            gripper.move(value[0],value[1],value[2])

if __name__=="__main__":
    main()
