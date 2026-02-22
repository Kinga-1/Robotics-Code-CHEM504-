import os
import sys
from examples.robotiq.robotiq_gripper import RobotiqGripper
import time


class gripper_handler:
    def __init__(self, ip:str, port:int):
        self.gripper_ip = ip
        self.gripper_port : int = port
        self.gripper = self._init_gripper()
    
    def _init_gripper(self):
        gripper = RobotiqGripper()
        gripper.connect(self.gripper_ip, self.gripper_port)
        return gripper
    
    def _test_gripper(self):
        for i in range(2):
              self.gripper.move(0,255,255) ## open
              time.sleep(0.5)
              self.gripper.move(225,255,255) ## close
              time.sleep(0.5)

    def close(self):
        self.gripper.move(225,255,255) ## close
        return True
    
    def open(self):
        self.gripper.move(0,255,255) ## open
    
    def grab_sample(self):
        self.gripper.move(140,255,255)