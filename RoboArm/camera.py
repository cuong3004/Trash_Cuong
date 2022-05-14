from object_detector import ObjectDetect
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera 
from PIL import Image
import numpy as np
import time
from arm import ArmController
import arm as rf
import sys
import RPi.GPIO as GPIO
import time
from PIL import Image, ImageOps
from imutils.video import VideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2



def padding(img, expected_size):
    desired_size = expected_size
    delta_width = desired_size - img.size[0]
    delta_height = desired_size - img.size[1]
    pad_width = delta_width // 2
    pad_height = delta_height // 2
    padding = (pad_width, pad_height, delta_width - pad_width, delta_height - pad_height)
    return ImageOps.expand(img, padding)


def resize_with_padding(frame, expected_size):
    img = Image.fromarray(frame)
    img.thumbnail((expected_size[0], expected_size[1]))
    delta_width = expected_size[0] - img.size[0]
    delta_height = expected_size[1] - img.size[1]
    pad_width = delta_width // 2
    pad_height = delta_height // 2
    pad_width = pad_height = 0
    padding = (pad_width, pad_height, delta_width - pad_width, delta_height - pad_height)
    return np.asarray(ImageOps.expand(img, padding))


class Conveyor:
    def __init__(self):

        PWD = 19

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PWD, GPIO.OUT)
        self.pwd = GPIO.PWM(PWD, 100)

        self.setspeed(0)
    
    def setspeed(self, speed):
        self.pwd.start(speed)

        
if __name__ == '__main__':
    vs = VideoStream(usePiCamera=True).start()
    time.sleep(2.0)
    # fps = FPS().start()

    frame_rate_calc = 1
    freq = cv2.getTickFrequency()
    font = cv2.FONT_HERSHEY_SIMPLEX

    object_detector = ObjectDetect()
    
    windowName = 'Object Detect'

    cv2.namedWindow(windowName)
    
    isPause = False
    
    arm = ArmController()
    
    convey = Conveyor()
    convey.setspeed(30)
    
    BTN1 = 14
    BTN3 = 3
    BTN4 = 2
    isbegin = True
    GPIO.setwarnings(False) 
    GPIO.setmode(GPIO.BCM) 
    GPIO.setup(BTN1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BTN3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BTN4, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
    conveyBool = False
    while True:
        
        if GPIO.input(BTN1) == GPIO.LOW:
            rf.button_press()
            time.sleep(0.25)
        
        if GPIO.input(BTN4) == GPIO.LOW:
            sys.exit()
            time.sleep(0.25)
            
        if GPIO.input(BTN3) == GPIO.LOW:
            conveyBool = not conveyBool
            if conveyBool:
                convey.setspeed(0)
            else:
                convey.setspeed(30)
            time.sleep(0.25)
        
        frame = vs.read()

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frameCut = frame[78:130, 130:208]
        frameCutPad = resize_with_padding(frameCut, (80, 80))
        frameCutPad = cv2.resize(frameCutPad, (128,128))
        
        t1 = cv2.getTickCount()

        boxes, scores, classes =  object_detector.detect(frameCutPad)

        y_max_list = []
        for i in range(len(scores)):
            if ((scores[i] > 0.5) and (scores[i] <= 1.0)):
                H = frameCutPad.shape[0]
                W = frameCutPad.shape[1]
        
                xmin = int(max(1,(boxes[i][0] * 80)))
                ymin = int(max(1,(boxes[i][1] * 80)))
                xmax = int(min(H,(boxes[i][2] * 80)))
                ymax = int(min(W,(boxes[i][3] * 80)))
                
                y_max_list.append(ymax)
                
                cv2.rectangle(frameCut, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)
        
        cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)

        if not isPause:
           if len(y_max_list) > 0:
               if max(y_max_list) > 35 :
                    convey.setspeed(0)
                    arm.picker(130)
                    time.sleep(1)
                    arm.drop()
                    convey.setspeed(25)
                    
        cv2.rectangle(frameCut, (0,0), (frameCut.T.shape[-2:]), (10, 0, 0), 2)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        cv2.imshow(windowName, frame)
        
        t2 = cv2.getTickCount()
        
        time1 = (t2-t1)/freq
        frame_rate_calc = 1/time1
        
        if cv2.waitKey(1) == ord('q'):
            break

        # fps.update()

    cv2.destroyAllWindows()
    vs.stop()
