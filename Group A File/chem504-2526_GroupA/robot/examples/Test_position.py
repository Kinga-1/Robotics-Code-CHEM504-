from PIL import ImageTk, Image
import numpy as np
import math
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
from utils.UR_Functions import URfunctions as URControl
# sys.path.append(os.path.join(current_dir, 'robotiq'))
# current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# print(f"current_dir : {current_dir}")
# sys.path.append(current_dir)


from utils.robotiq_gripper import RobotiqGripper



HOST = "192.168.0.2"
PORT = 30003
# from robotiq.robotiq_gripper import RobotiqGripper
def main():
    robot = URControl(ip="192.168.0.2", port=30003)
    HOST = "192.168.0.2"
    PORT = 30003
    gripper=RobotiqGripper()
    gripper.connect("192.168.0.2", 63352)
    gripper.move(255,125,125)
#     joint_state=degreestorad([93.77,-89.07,89.97,-90.01,-90.04,0.0])
#     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
    
#     joint_state=[1.0940968990325928, -1.1060843926719208, 1.6807625929461878, -2.098030229608053, -1.6097186247455042, -0.3137314955340784]
#     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
#     gripper.move(140,255,255)
    
    
    
def degreestorad(list):
     for i in range(6):
          list[i]=list[i]*(math.pi/180)
     return(list)    
 

if __name__=="__main__":
     main()