from examples.utils.UR_Functions import URfunctions as URControl
from utils.coordinate_utility import coordinate_util as coord_util

HOME = [1.3651890754699707, -1.5046540063670655, 1.8308394590960901, -1.913860937158102, -1.567221466694967, 2.9111227989196777]

SLOW_ACC = 0.25
SLOW_SPEED = 0.8
ROT = 0.02
FAST_ACC = 1
FAST_SPEED = 2.5


class robot(URControl):
    # handles robot movements and interactions.
    def __init__(self, ip: str, port: int):
        super().__init__(ip, port)
        self.ip_addr = ip
        self.port = port
        self.current_position = HOME
        print(self.get_current_joint_positions().tolist())

    def return_home(self):
        # moves the robot to its home position.
        self.go_home()
    
    def get_joints(self):
        # returns the current joint positions of the robot.
        return self.get_current_joint_positions()
    
    def move_joints_fast(self, position: list):
        # moves the robot to a joint position quickly.
        self.move_joint_list(position,  FAST_SPEED, FAST_ACC, ROT)

    def move_joints_slow(self, position: list):
        # moves the robot to a joint position slowly.
        self.move_joint_list(position,  SLOW_SPEED, SLOW_ACC, ROT)