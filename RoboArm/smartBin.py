
import time

from pyrf24 import RF24
from nrf24 import NRF24
BROCKER_ADDRESS = 'localhost'

OBJECT_DETECT_TOPIC = "/object"

ROBOT_STATUS = "/status"

NRF24_ADDRESS = 0xF0F0F0F0D2

SERVO1 = 19
SERVO2 = 12
SERVO3 = 18
SERVO4 = 13

BTN = 31


class NRF24L01:
    def __init__(self):
        self.radio = NRF24(22, 0)

        self.radio.begin()
        self.radio.setRetries(5,15)
        self.radio.dynamic_payloads = True
        self.radio.ack_payloads = True
        self.radio.setDataRate(RF24_250KBPS)

        self.radio.openWritingPipe(NRF24_ADDRESS)

        self.radio.powerUp()
    
    def write(self, data):
        self.radio.write(data)
        



rf24 = NRF24L01()

rf24.write(b'11')
rf24.write(b'21')
rf24.write(b'31')
