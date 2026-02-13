import sys
import os
import time
import argparse
import math 
# Add the directory containing robotiq_preamble.py to the Python search path
current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
print(current_dir)
sys.path.append(current_dir)


from utils.robotiq_gripper import RobotiqGripper

HOST = "192.168.0.2"
PORT = 30003

def main():
    #tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #tcp_socket.connect((HOST, PORT))
    gripper=RobotiqGripper()
    gripper.connect(HOST, 63352)
    #gripper.activate()
    gripper.move(140,255,255)
    time.sleep(10)
    gripper.move(gripper.get_open_position(),255,0)


if __name__ == '__main__':
    main()