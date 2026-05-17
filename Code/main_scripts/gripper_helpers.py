def OpenGripper(gripper):
    # move gripper to fully open position.
    if gripper:
        gripper.move(0, 125, 125)
        print("Gripper: Opened")

def CloseGripper(gripper):
    # move gripper to fully closed position.
    if gripper:
        gripper.move(240, 125, 125)
        print("Gripper: Closed")