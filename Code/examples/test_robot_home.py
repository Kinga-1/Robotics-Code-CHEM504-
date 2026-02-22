# -*- encoding:utf-8 -*-
import sys
import os
import time
import argparse
import math 
# Add the directory containing robotiq_preamble.py to the Python search path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, 'robotiq'))

from utils.UR_Functions import URfunctions as URControl

HOST = "192.168.0.2"
PORT = 30003

def main():
    robot = URControl(ip="192.168.0.2", port=30003)
    #robot.go_home()

    joint_state=degreestorad([93.77,-89.07,89.97,-90.01,-90.04,0.0])
    robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

    joint_state = [0.00000744, -1.57083954, 1.57082969, -1.57077511, -1.57079918, -0.00003463]
    #robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

    joint_state = [1.6898278, -0.118682, -2.8710666, -1.7318902, 0.009250245, 0.001919862]
    #robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
    #FOR ROS  
    joint_state = [0.0,-1.57,0.0,-1.57,0.0,0.0]
    #robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

def degreestorad(list):
     for i in range(6):
          list[i]=list[i]*(math.pi/180)
     return(list)

if __name__ == '__main__':
     main()