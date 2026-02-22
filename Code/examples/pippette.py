from PIL import ImageTk, Image
import numpy as np
import math
import os
import sys
import time
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, 'robotiq'))
from utils.UR_Functions import URfunctions as URControl
from robotiq.robotiq_gripper import RobotiqGripper
def main():
    robot = URControl(ip="192.168.0.2", port=30003)
    gripper=RobotiqGripper()
    gripper.connect("192.168.0.2", 63352)
    gripper.move(255,125,125)
    joint_state = [ 2.07629466, -1.17086525,  1.06810648, -1.43957524, -1.54322559,  0.3712894]
    robot.move_joint_list(joint_state, 0.05, 0.25, 0.02)
    joint_state = [2.07626724, -1.173539,    1.24181921, -1.61063733, -1.5434416,   0.37211591]
    robot.move_joint_list(joint_state, 0.05, 0.25, 0.02)
    gripper.move(210,0,0)
    time.sleep(3)
    joint_state = [ 2.07629466, -1.17086525,  1.06810648, -1.43957524, -1.54322559,  0.3712894]
    robot.move_joint_list(joint_state, 0.05, 0.25, 0.02)
    joint_state = [2.32826447, -1.09934129,  1.03138191, -1.45263104, -1.59819729,  0.75525182]
    robot.move_joint_list(joint_state, 0.05, 0.25, 0.02)
    time.sleep(1)
    gripper.move(255,125,125)


    
    


   
if __name__ == '__main__':
    main()