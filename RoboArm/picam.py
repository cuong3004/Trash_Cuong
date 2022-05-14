from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np

IM_WIDTH = 480
IM_HEIGHT = 320

if __name__ == '__main__':
    
    camera = PiCamera()
    camera.resolution = (IM_WIDTH,IM_HEIGHT)
    camera.framerate = 1
    rawCapture = PiRGBArray(camera, size=(IM_WIDTH,IM_HEIGHT))
    rawCapture.truncate(0)

    for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
        t1 = cv2.getTickCount()

        frame = np.copy(frame1.array)
        frame.setflags(write=1)

        if cv2.waitKey(1) == ord('q'):
            break

        cv2.imshow('Object detector', frame)
        rawCapture.truncate(0)
        
    camera.close()