from handlers.gripper_handler import gripper_handler as Gripper
from handlers.robot_handler import robot as Robot

APPROACH_POSE = [0.878345251083374, -1.4129328739694138, 1.9044411818133753, -2.091734548608297, -1.4622834364520472, -2.130199734364645]
PICK_POSE = [0.9737541675567627, -1.2341589492610474, 1.9380624930011194, -2.3127924404540003, -1.5530193487750452, 2.515105724334717]
class pick_sample_subroutine():
    def __init__(self, robot: Robot, gripper :Gripper):
        self.robot = robot
        self.gripper = gripper

    def run(self):
        self.robot.move_joints_fast(APPROACH_POSE)
        self.robot.move_joints_slow(PICK_POSE)
        self.gripper.grab_sample()
        self.robot.move_joints_slow(APPROACH_POSE)