CAMERA_POSE = [2.4489846229553223, -1.2780983012965699, 1.3398712317096155, -1.6268969974913539, -1.5511811415301722, 3.849146604537964]

class vial_shake_generator:
    def __init__(self, rotation: float , number_cycles: int, start_position: list):
        self.rotation = rotation
        self.number_shakes = number_cycles *2
        self.shake_coords : list = self._generate_shake_coord()
        self.start_pose = CAMERA_POSE
    
    def _generate_shake_coord(self):
        
        ## this will generate a series of join positions that makes 1 cycle of shaking
        shake_coords : list = []
        test_shake = []
        for i in range(self.number_shakes ):
            test_shake = [2.7035980224609375, -1.1756010812572022, 1.2089040915118616, -1.6375476322569789, -1.5411441961871546, 4.249309539794922]
            if (i%2==0):
                test_shake = [2.5206339359283447, -1.0861122471145173, 2.033088986073629, -4.1367989979186, -0.9123547712909144, 4.7877888679504395]
            shake_coords.append(test_shake)
        return shake_coords

    
