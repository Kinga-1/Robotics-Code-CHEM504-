import sys
import os
import time
from datetime import datetime
import argparse
import math 
import os
import sys
import cv2
# Add the directory containing robotiq_preamble.py to the Python search path


# current_dir = os.path.dirname(os.path.abspath(__file__))
# sys.path.append(os.path.join(current_dir, 'robotiq'))
# print(current_dir)
from utils.UR_Functions import URfunctions as URControl, RobotiqGripper

# current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# print(current_dir)
# sys.path.append(current_dir)
# from utils.robotiq_gripper import RobotiqGripper

current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
print(current_dir)
sys.path.append(current_dir)


from handlers.camera_handler import Camera

HOST = "192.168.0.2"
PORT = 30003

def main():
    robot = URControl(ip="192.168.0.2", port=30003)

    print(robot.get_current_joint_positions().tolist())
    print(robot.get_current_tcp())
    
    # gripper=RobotiqGripper()
    # gripper.connect(HOST, 63352)
    
    # cam = cv2.VideoCapture(0)
    # ret, frame = cam.read()
    # img_counter = 0
    # cv2.imwrite('test.png', frame)
    
    # HOME_TCP = [-0.16189536, -0.45307684,  0.14772634,  0.10644612, -3.13254266, -0.02294322]
    
    
    # robot.movej_tcp(HOME_TCP, 0.2, 0.5)
    # time.sleep(1)
    # robot.movej_tcp(PICK_ABOVE_TCP, 0.2, 0.5)
    # time.sleep(1)
    # robot.movej_tcp(PICK_TCP, 0.2, 0.5)
    # time.sleep(1)
    
    # gripper.move(140,255,255)
    # time.sleep(1)
    
    # robot.movej_tcp(PICK_ABOVE_TCP, 0.2, 0.5)
    # robot.go_home()
    # for i in range(10):
    #     a = datetime.now()
    #     print(robot.get_current_tcp())
    #     print(datetime.now() - a)
    # current_position = robot.get_current_tcp()
    # current_position[3:6] = HOME_TCP[3:6]
    # print("----------------Start ------------")
    # while True:
    #     _pos = robot.get_current_tcp()
    #     print(f"time :{datetime.now()}, pos {_pos}")
        
        
    
if __name__ == '__main__':
     main()