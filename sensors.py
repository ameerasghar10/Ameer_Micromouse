from machine import I2C, Pin, SoftI2C
from vl6180x_sensor import Sensor
import time
#This initialises the I2C buses for the ToF sensors on the pico
class Sensors:
    def __init__(self):
        self.i2c_front = I2C(0, scl=Pin(1), sda=Pin(0))   
        self.i2c_left = I2C(1, scl=Pin(3), sda=Pin(2))    
        self.i2c_right = SoftI2C(scl=Pin(5), sda=Pin(4))   

        self.sensor_front = Sensor(self.i2c_front)
        self.sensor_left = Sensor(self.i2c_left)
        self.sensor_right = Sensor(self.i2c_right)

    def read_distances(self):
        distance_front = self.sensor_front.range()
        distance_left = self.sensor_left.range()
        distance_right = self.sensor_right.range()
        return distance_front, distance_left, distance_right
