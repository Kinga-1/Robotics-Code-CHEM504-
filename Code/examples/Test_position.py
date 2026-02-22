from PIL import ImageTk, Image
import numpy as np
import math
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
from utils.UR_Functions import URfunctions as URControl
from positions_list import PositionList
# sys.path.append(os.path.join(current_dir, 'robotiq'))
# current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# print(f"current_dir : {current_dir}")
# sys.path.append(current_dir)


from utils.robotiq.robotiq_gripper import RobotiqGripper



HOST = "192.168.0.2"
PORT = 30003


# from robotiq.robotiq_gripper import RobotiqGripper
def main():
     robot = URControl(ip="192.168.0.2", port=30003)
     HOST = "192.168.0.2"
     PORT = 30003
     gripper=RobotiqGripper()
     gripper.connect("192.168.0.2", 63352)
     #print(#gripper positon)
     OpenGripper(gripper)
         
    

# Home position
     joint_state = PositionList.defaultPos 
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
    
# Hover above vial 0 
     joint_state = PositionList.aboveVialRack
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
     OpenGripper(gripper)

# Grip vial 1
     joint_state = PositionList.onVialRack
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
     CloseGripper(gripper)

# Hover with vial 1
     joint_state = PositionList.aboveVialRack
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Filming position (white background) 1
     joint_state = PositionList.filmingPos
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Hover above stirring plate 1
     joint_state = PositionList.aboveStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Place on stirring plate 1
     joint_state = PositionList.onStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# open gripper 
     OpenGripper(gripper)
     
# Hover above stirring plate 0
     joint_state = PositionList.aboveStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Pick up vial from stirring plate 0
     joint_state = PositionList.onStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
     
# close gripper
     CloseGripper(gripper)

# Hover above stirring plate 0
     joint_state = PositionList.aboveStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Filming position (white background) 1
     joint_state = PositionList.filmingPos
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Hover above stirring plate 1
     joint_state = PositionList.aboveStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Place on stirring plate 1
     joint_state = PositionList.onStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# open gripper 
     OpenGripper(gripper)
     
# Hover above stirring plate 0
     joint_state = PositionList.aboveStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Pick up vial from stirring plate 0
     joint_state = PositionList.onStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
     
# close gripper
     CloseGripper(gripper)

# Filming position (white background) 1
     joint_state = PositionList.filmingPos
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
     
# Hover above vial rack 1
     joint_state = PositionList.aboveVialRack
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# place on vial rack 0
     joint_state = PositionList.onVialRack
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
     
# open gripper
     OpenGripper(gripper)

# Hover above vial 0 
     joint_state = PositionList.aboveVialRack
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Home position
     joint_state = PositionList.defaultPos 
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
     OpenGripper(gripper)


    
def degreestorad(list):
     for i in range(6):
          list[i]=list[i]*(math.pi/180)
     return(list)    
 
def OpenGripper(gripper):
     gripper.move(0,125,125)
     print("Opened")
     
def CloseGripper(gripper):
     gripper.move(140,125,125)
     print("Closed")
 

if __name__=="__main__":
     main()
     
     
     