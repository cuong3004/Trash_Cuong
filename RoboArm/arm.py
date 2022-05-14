import pigpio
import time

import pigpio
import time

from nrf24l01 import NRF24L01


trashbin = False

rf24 = NRF24L01()

def button_press():
    global trashbin
    trashbin = not trashbin
    if trashbin:
        for _ in range(3):
            rf24.write(b'11') #trash ID = 1; Open command = 1
            rf24.write(b'21') #trash ID = 2; Open command = 1
            rf24.write(b'31') #trash ID = 3; Open command = 1
            time.sleep(0.1)
    else:
        for _ in range(3):
            rf24.write(b'10') #trash ID = 1; Close command = 0
            rf24.write(b'20') #trash ID = 2; Close command = 0
            rf24.write(b'30') #trash ID = 3; Close command = 0
            time.sleep(0.1)

L1 = 13.5 #chieu dai cac khop canh tay
L2 = 14.7 #L1 = thang; L2 = ngang
L = 20 #L = tam den bang truyen
L0 = 20


trashID = 1
if trashID == 1:
    pulseServo1 = 1300
    writeData_open = b'11'
    writeData_close = b'10'
if trashID == 2:
    pulseServo1 = 1000
    writeData_open = b'21'
    writeData_close = b'20'
if trashID == 3:
    pulseServo1 = 500
    writeData_open = b'31'
    writeData_close = b'30'

class ArmController:
    def __init__(self):
        self.pi = pigpio.pi()
        
        self.SERVO1 = 21
        self.SERVO2 = 20
        self.SERVO3 = 6
        self.SERVO4 = 12
        print("Khoi dong")
        self.servoPos = {}
        self.get_start()
        

    def picker(self, x_pos):
        
            
        self._setGpio(21, 2500) # quay servo1 den bang chuyen

        for i in np.arange(11,-1.0,-0.3):

            L = equaltion_line_(i)

            belta = math.acos((L1**2+L2**2-L**2)/(2*L1*L2))
            alpha = math.acos((L*L + L1*L1 - L2*L2 )/(2*L1*L))

            theta = math.atan2(i,L0)

            if i == 1:
                time.sleep(0.5)
                
            print("SERVO2", anglepredict2Pulse(radi2angle(alpha+theta)) - 1.3*i**2-27)
            print("SERVO3", anglepredict2Pulse2(radi2angle(belta)))

            self._setGpio(20, anglepredict2Pulse(radi2angle(alpha+theta)) - 1.3*i**2-27, step=5)
            self._setGpio(6, anglepredict2Pulse2(radi2angle(belta))-25, step=5)
            
        # self._setGpio(12, 1800)
        
        time.sleep(1)
        self._setGpio(12, 500)  
        for i in np.arange(1,13,0.3):

            L = equaltion_line_(i)

            belta = math.acos((L1**2+L2**2-L**2)/(2*L1*L2))
            alpha = math.acos((L*L + L1*L1 - L2*L2 )/(2*L1*L))

            theta = math.atan2(i,L0)

            print(anglepredict2Pulse(radi2angle(alpha+theta))-i*12)

            self._setGpio(20, anglepredict2Pulse(radi2angle(alpha+theta)) - 1.3*i**2-27, step=5)
            self._setGpio(6, anglepredict2Pulse2(radi2angle(belta)), step=5)

            if i == 1:
                time.sleep(1)
                
        
        rf24.write(writeData_open)
        time.sleep(0.5)
        
        
        
    
    def drop(self):
        
        
        
        self._setGpio(21, pulseServo1)
        time.sleep(1)
        
        for i in np.arange(9,7.3,-0.3):

            L = equaltion_line_(i)


            belta = math.acos((L1**2+L2**2-L**2)/(2*L1*L2))
            alpha = math.acos((L*L + L1*L1 - L2*L2 )/(2*L1*L))

            theta = math.atan2(i,L0)

            if i == 1:
                time.sleep(0.5)
                
            
            if trashID == 1:
                self._setGpio(20, anglepredict2Pulse(radi2angle(alpha+theta)) - 1.3*i**2-27 + 3*(15-i)**2, step=5)
                self._setGpio(6, anglepredict2Pulse2(radi2angle(belta))+(15-i)**2, step=5)
            else:
                self._setGpio(20, anglepredict2Pulse(radi2angle(alpha+theta)) - 1.3*i**2-27, step=5)
                self._setGpio(6, anglepredict2Pulse2(radi2angle(belta)), step=5)

            
        time.sleep(0.5)
        
        self._setGpio(12, 1800)  
        for i in np.arange(7,15,0.3):

            L = equaltion_line_(i)

            belta = math.acos((L1**2+L2**2-L**2)/(2*L1*L2))
            alpha = math.acos((L*L + L1*L1 - L2*L2 )/(2*L1*L))

            theta = math.atan2(i,L0)

            #print(anglepredict2Pulse(radi2angle(alpha+theta))-i*12)
            print("radi2angle(belta)", anglepredict2Pulse2(radi2angle(belta)))
            
            if trashID == 1:
                self._setGpio(20, anglepredict2Pulse(radi2angle(alpha+theta)) - 1.3*i**2-27 + 3*(15-i)**2, step=5)
                self._setGpio(6, anglepredict2Pulse2(radi2angle(belta))+(15-i)**2, step=5)
            else:
                self._setGpio(20, anglepredict2Pulse(radi2angle(alpha+theta)) - 1.3*i**2-27, step=5)
                self._setGpio(6, anglepredict2Pulse2(radi2angle(belta)), step=5)

            if i == 1:
                time.sleep(1)
        
        
        rf24.write(writeData_close)
        
        self._setGpio(21, 2500)
        time.sleep(1)
    
    def pulse2Angle(self, pulse):
        return -0.1 * pulse + 240
    
    def angle2Pulse(self, angle):
        return round(-10 * angle + 2400)
    
    # (-10 * angle + 2400)
    
    def get_start(self):
        
        self.servoPos[self.SERVO1] = 2300
        self.servoPos[self.SERVO2] = 1300
        self.servoPos[self.SERVO3] = 1850
        self.servoPos[self.SERVO4] = 1800
        """
        self._setGpio(self.SERVO1, 1500)
        time.sleep(1)
        self._setGpio(self.SERVO2, 1300)
        time.sleep(1)
        self._setGpio(self.SERVO3, 1850)
        time.sleep(1)
        self._setGpio(self.SERVO4, 1500)
        time.sleep(1)
        """
        self.pi.set_servo_pulsewidth(self.SERVO1, 2300)
        time.sleep(1)
        self.pi.set_servo_pulsewidth(self.SERVO2, 1300)
        time.sleep(1)
        self.pi.set_servo_pulsewidth(self.SERVO3, 1850)
        time.sleep(1)
        self.pi.set_servo_pulsewidth(self.SERVO4, 1800)
        

        print(self.servoPos)
    
    
        
        
    
    def _setGpio(self, pin, pulse_width, smooth = True, step = 10, release = True):
        if pulse_width < 500 or pulse_width > 2500:
            return
        if smooth:
            pulse = self.servoPos[pin]
            
            if pulse < pulse_width:
                
                while pulse < pulse_width:
                    pulse = pulse + step
                    # print(pulse)
                    self.pi.set_servo_pulsewidth(pin, pulse)
                    time.sleep(0.004)
            else:
                while pulse > pulse_width:
                    pulse = pulse - step
                    self.pi.set_servo_pulsewidth(pin, pulse)
                    time.sleep(0.004)

            self.pi.set_servo_pulsewidth(pin, pulse_width)
            time.sleep(0.004)
            
            self.servoPos[pin] = pulse_width
            if release:
                self.pi.set_servo_pulsewidth(pin, 0)
            
        else:
            # print(pulse_width)
            self.pi.set_servo_pulsewidth(pin, pulse_width)
            sleep_time = abs(self.servoPos[pin] - pulse_width) / 1000 * 1.0
            time.sleep(sleep_time)
            self.servoPos[pin] = pulse_width
            
            if release:
                self.pi.set_servo_pulsewidth(pin, 0)
                
                
    def originState(self, smooth = True):
        self.current_pos = [900, 600, 1500, 1500]

        self._setGpio(SERVO4, current_pos[3], smooth, step=20)
        self._setGpio(SERVO2, current_pos[2], smooth, step=20)
        self._setGpio(SERVO3, current_pos[1], smooth, step=20)
        self._setGpio(SERVO1, current_pos[0], smooth, step=20)


import math
import numpy as np
L1 = 13.5
L2 = 14.7
L = 20

def equal_convey(x):
    return (67/59)*x+(6800/59)

def radi2angle(radi):
    return 57.2957795*radi

def angle2Pulse(angle):
        return round((50/3)*(180-angle) + 500)


def anglepredict2Pulse(angle):
    return round((-100/9)*angle+2200)

def anglepredict2Pulse2(angle):
    return round((50/3)*angle)
    
def equaltion_line_(idx):
    return (1/3)*idx + 59/3




if __name__ == "__main__":
    arm = ArmController()
