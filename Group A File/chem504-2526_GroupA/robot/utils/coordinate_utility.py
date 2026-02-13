import math

class coordinate_util:
    def __init__(self):
       pose = [0.5217525959014893, -0.8984154027751465, 2.2390645186053675, -2.453958650628561, -0.0081713835345667, -1.974764649068014] 
        
    def degreestorad(list):
        for i in range(6):
            list[i]=list[i]*(math.pi/180)
        return(list)   
    
    def get_test_poses():
        test_poses = []

        for i in range(8):
            pose = [0.5217525959014893, -0.8984154027751465, 2.2390645186053675, -2.453958650628561, -0.0081713835345667, -1.974764649068014]
            test_poses.append(pose)
        return test_poses
    
    def get_test_set_pose():
        test_pose = []
        for i in range(8):
            pose = [1.2957301139831543, -0.9064623278430481, 1.8208301703082483, -1.1233906608871003, -0.22048646608461553, -2.9866870085345667]
            test_pose.append(pose)
        return test_pose