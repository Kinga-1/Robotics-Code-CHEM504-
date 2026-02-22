import cv2
import numpy as np
import time

class Camera:
    def __init__(self, port: int, sample_name : str):
        #self.videoController = cv2.VideoCapture(port)
        self.finish : bool = False
        self.name = sample_name 
       
    def capture_video(self):
        cap = cv2.VideoCapture(0) 
        if (cap.isOpened()==False):
            print("eror opening camera")
        
        print("Starting record")
        while (cap.isOpened()):
            capture_duration=10
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            out = cv2.VideoWriter(self.name + '.avi', fourcc, 20.0, (640, 480))
            start_time = time.time()
            while (int(time.time() - start_time)< capture_duration):
                ret,frame = cap.read()
                if ret==True:
                    frame=cv2.flip(frame,0)
                    out.write(frame)
                
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH) 
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            cap.release()
            out.release()  
            cv2.destroyAllWindows()
            print("finsihed recording")
            if cv2.waitKey(25) & 0xFF ==ord('q'):
                break

    def capture_image(self, filename):
        cam = cv2.VideoCapture(0)
        cv2.namedWindow(filename)
        ret, frame = cam.read()
        img_counter = 0
        cv2.imwrite(filename, frame)
        cam.release
        
