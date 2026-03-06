from PIL import ImageTk, Image
import numpy as np
import math
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
from utils.UR_Functions import URfunctions as URControl
from positions_list import PositionsList
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
     gripper.move(255,125,125)
    

# Home position
     joint_state = PositionsList.defaultPo 
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
#     --> we need to define open and close positons gripper.move(gripper.get_open_position(), 255, 0)
    
# Hover above vial 0 
     joint_state = PositionsList.aboveVialRack
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
#     gripper.move(140,255,255) --> gripper open 

# Grip vial 1
     joint_state = PositionsList.onVialRack
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
#     gripper.move(140,255,255) --> close gripper

# Hover with vial 1
     joint_state = PositionsList.aboveVialRack
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Filming position (white background) 1
     joint_state = PositionsList.filmingPos
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Hover above stirring plate 1
     joint_state = PositionsList.aboveStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Place on stirring plate 1
     joint_state = PositionsList.onStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# open gripper 

     
# Hover above stirring plate 0
     joint_state = PositionsList.aboveStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Pick up vial from stirring plate 0
     joint_state = PositionsList.onStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
     
# close gripper

# Hover above stirring plate 0
     joint_state = PositionsList.aboveStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Filming position (white background) 1
     joint_state = PositionsList.filmingPos
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Hover above stirring plate 1
     joint_state = PositionsList.aboveStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Place on stirring plate 1
     joint_state = PositionsList.onStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# open gripper 

     
# Hover above stirring plate 0
     joint_state = PositionsList.aboveStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Pick up vial from stirring plate 0
     joint_state = PositionsList.onStirringPlate
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# Filming position (white background) 1
     joint_state = PositionsList.filmingPos
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
     
# Hover above vial rack 1
     joint_state = PositionsList.aboveVialRack
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)

# place on vial rack 0
     joint_state = PositionsList.onVialRack
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
     
# open gripper

# Hover above vial 0 
     joint_state = PositionsList.aboveVialRack
     robot.move_joint_list(joint_state, 0.5, 0.5, 0.02)
  
# Home position
     joint_state = PositionsList.defaultPos 


    
def degreestorad(list):
     for i in range(6):
          list[i]=list[i]*(math.pi/180)
     return(list)    
 

if __name__=="__main__":
     main()
     
     
     