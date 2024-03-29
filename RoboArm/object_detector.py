import numpy as np

import cv2
import tflite_runtime.interpreter as tflite
import time


class NMS:
    def __init__(self):
        self.interpreter = tflite.Interpreter('trash_v3/nms.tflite')
        self.interpreter.allocate_tensors()
        
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
    def __call__(self, output_data):
        self.interpreter.set_tensor(self.input_details[0]['index'], output_data)
        self.interpreter.invoke()
        
        output_bboxes = self.interpreter.get_tensor(self.output_details[1]['index'])
        output_scores = self.interpreter.get_tensor(self.output_details[0]['index'])
        output_classes = self.interpreter.get_tensor(self.output_details[2]['index'])
        
        return output_bboxes, output_scores, output_classes

class ObjectDetect:
    def __init__(self):
        with open("labels.txt", "r") as f:
            self.label_map = [i.replace("\n", "") for i in f.readlines()]
        
        self.nms = NMS()
            
            
        self.interpreter = tflite.Interpreter(#"trash_v3/best-int8.tflite"
                                                'trash_v3/best-int8_edgetpu.tflite',
                                              experimental_delegates=[tflite.load_delegate('libedgetpu.so.1.0')]
                                              )
        self.interpreter.allocate_tensors()
        
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
    def detect(self, image):
        image = image / 255
        frame_expanded = np.expand_dims(image, axis=0)
        
        scale, zero_point = self.input_details[0]['quantization']
        frame_expanded = (frame_expanded / scale + zero_point).astype(np.uint8)
        self.interpreter.set_tensor(self.input_details[0]['index'], frame_expanded)
        self.interpreter.invoke()
        
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        scale, zero_point = self.output_details[0]['quantization']
        output_data = (output_data.astype(np.float32) - zero_point) * scale
        
        boxes, scores, classes = self.nms(output_data)

        return boxes, scores, classes 
    

if __name__ == "__main__":
    from PIL import Image
    import numpy as np
    
    nms = ObjectDetect()
    
    print(nms.input_details)
