import requests
from PIL import ImageTk, Image
import numpy as np
import matplotlib.image
import shutil
import io
import cv2 as cv
import sys,os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, 'robotiq'))
from utils.UR_Functions import URfunctions as URControl
from robotiq.robotiq_gripper import RobotiqGripper
def main():
    resp = requests.get("http://"+"192.168.0.2"+":4242/current.jpg?type=color",stream=True)
    robot = URControl(ip="192.168.0.2", port=30003)
    joint_state=([1.60862231, -0.68330659,  0.85918743, -1.12231465, -1.56227094,  0.00816095])
    robot.move_joint_list(joint_state, 0.1, 0.1, 0.02)
    

if __name__=="__main__":
    main()