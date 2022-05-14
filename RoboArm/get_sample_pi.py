
from imutils.video import VideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2
import numpy as np

def draw_detect(boxes, scores, classes, frame):
    for i in range(len(scores)):
        if ((scores[i] > 0.5) and (scores[i] <= 1.0)):
            H = frame.shape[0]
            W = frame.shape[1]
    
            xmin = int(max(1,(boxes[i][0] * W)))
            ymin = int(max(1,(boxes[i][1] * H)))
            xmax = int(min(H,(boxes[i][2] * W)))
            ymax = int(min(W,(boxes[i][3] * H)))
            
            # x_list.append((xmin+xmax)/2)
            
            cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)
                
        
from PIL import Image, ImageOps


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

def notthing(value):
    pass


if __name__ == '__main__':

    vs = VideoStream(usePiCamera=True).start()
    time.sleep(2.0)
    fps = FPS().start()

    windowName = 'Object Detect'

    cv2.namedWindow(windowName)

    # cv2.createTrackbar('sliderWidth', windowName, 0, 320, notthing)
    # cv2.createTrackbar('sliderHeight', windowName, 0, 240, notthing)
    idx_frame_save = 0
    while True:
        
        frame = vs.read()

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # frame = cv2.resize(frame, (640, 480))
        frameCut = frame[78:130, 130:208]
        frameCutPad = resize_with_padding(frameCut, (80, 80))
        # print(frameCut.shape)
        cv2.rectangle(frameCut, (0,0), (frameCut.T.shape[-2:]), (10, 0, 0), 2)
        # frameCut = frame

        # frameCut = Image.fromarray(frameCut)
        # frameCut = resize_with_padding(frameCut, (150, 150))
        # frameCut = np.asarray(frameCut)

        # widthPos = cv2.getTrackbarPos('sliderWidth',windowName)
        # heightPos = cv2.getTrackbarPos('sliderHeight',windowName)

        # frame = cv2.circle(frame, (widthPos,heightPos), radius=2, color=(0, 0, 255), thickness=-1)
        
        
        # translated = imutils.translate(frameCut, 25, -75)

        # cv2.rectangle(translated, (0,0), (50, 20), (10, 0, 0), 5)
        
        # frameResize = cv2.resize(frameCut, (320,320))

        
        frameCutPad = cv2.cvtColor(frameCutPad, cv2.COLOR_RGB2BGR)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imshow(windowName, frameCutPad)
        cv2.imshow(windowName + "02", frame)
        
        if cv2.waitKey(1) == ord('q'):
            name_img = f"image/image_{idx_frame_save}.png"
            cv2.imwrite(name_img, frameCutPad)
            idx_frame_save += 1
            print(name_img)
            time.sleep(0.1)
            # break
        # fps.update()
    fps.stop()
    cv2.destroyAllWindows()
    vs.stop()
