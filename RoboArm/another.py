import RPi.GPIO as GPIO
import time

class Conveyor:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        
        PWD = 19
        
        GPIO.setup(PWD, GPIO.OUT)
        
        self.pwd = GPIO.PWM(PWD, 100)
        
        
        self.setspeed(0)
    
    def setspeed(self, speed):
        self.pwd.start(speed)

if __name__ == "__main__":
    convey = Conveyor()
    
    convey.setspeed(30)
    # time.sleep(5)
    while True:
        pass
    
    #convey.setspeed(0)
    
    