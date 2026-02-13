from utils.vial_shake_generator import vial_shake_generator as vsg
import time
import threading
from multiprocessing import Process
from datetime import datetime
from utils.coordinate_utility import coordinate_util as coord_util
from handlers.robot_handler import robot as Robot
from handlers.camera_handler import Camera 

PRE_POSE  = test_shake = [2.7035980224609375, -1.1756010812572022, 1.2089040915118616, -1.6375476322569789, -1.5411441961871546, 4.249309539794922]
CAMERA_POSE = [2.5206339359283447, -1.0861122471145173, 2.033088986073629, -4.1367989979186, -0.9123547712909144, 4.7877888679504395]
## this subrouting extends the robot class (it has all the methods and attributes and can make more)
class vial_shake_subroutine():
    def __init__(self, number_cycles : int, robot: Robot):
        self.robot : Robot = robot ## this would be a pointer to the robot object IF I HAD POINTERS
        #self.start_point = self.robot.current_position ## this will get the current joint positions from the robot
        self.start_point = CAMERA_POSE
        self.shake_cycles : int  = number_cycles
        self.vsg = vsg(1.57,number_cycles,self.start_point) ## this uses the helper function to generate the shake positions
    
    def shake_vial(self, sample_number : int, type: str):
        ## the robot subroutine that shakes the vial in space (gets its own start position)
        base_filename = str(sample_number) + type
        
        self.robot.move_joints_fast(PRE_POSE)
        print("Im in place")
        time.sleep(1)
        for i in range(self.shake_cycles):
            print(f"Cycle number : {i}" )
            camera = Camera(0, base_filename)
            camera.name = str(datetime.today().strftime('%Y-%m-%d')) + type + str(sample_number)
            t1=threading.Thread(target=camera.capture_video, args=())
            print("moving to pre-pose")
            self.robot.move_joints_fast(PRE_POSE)
            
            print("doing tilt")
            t1.start()
            
            self.robot.move_joints_fast(CAMERA_POSE)
            time.sleep(5)
        return camera.name
        
            
            
            
            
            
        

    