from handlers.gripper_handler import gripper_handler as Gripper
from handlers.robot_handler import robot as Robot

APPROACH_POSE = [0.9834573268890381, -1.3181231778911133, 1.852170769368307, -2.1323305569090785, -1.586179558430807, 2.5227088928222656]
PICK_POSE = [0.9715018272399902, -1.2166730624488373, 1.8871625105487269, -2.215367456475729, -1.554498020802633, 2.4757308959960938]
class pick_sample_subroutine():
    def __init__(self, robot: Robot, gripper :Gripper):
        self.robot = robot
        self.gripper = gripper

    def run(self):
        self.robot.move_joints_fast(APPROACH_POSE)
        self.robot.move_joints_slow(PICK_POSE)
        self.gripper.grab_sample()
        self.robot.move_joints_slow(APPROACH_POSE)