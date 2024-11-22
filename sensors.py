from machine import Pin, I2C, time_pulse_us
import time
from config import TOF_LEFT_SDA, TOF_LEFT_SCL, TOF_RIGHT_SDA, TOF_RIGHT_SCL
from config import ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN
from vl6180x_sensor import Sensor

class Sensors:
    def __init__(self):

# This defines the I2C communication for the left and right ToF sensors and uses 0 and 1 for the 2 different channels that the Pico W has
        self.i2c_left = I2C(0, scl=Pin(TOF_LEFT_SCL), sda=Pin(TOF_LEFT_SDA))
        self.i2c_right = I2C(1, scl=Pin(TOF_RIGHT_SCL), sda=Pin(TOF_RIGHT_SDA))
        self.tof_left = Sensor(self.i2c_left)
        self.tof_right = Sensor(self.i2c_right)
        
# This defines the ultrasonic sensor pins (Trig is used for sending out the pulse and Echo is used for receiving the echo back)
        self.TRIG = Pin(ULTRASONIC_TRIG_PIN, Pin.OUT)
        self.ECHO = Pin(ULTRASONIC_ECHO_PIN, Pin.IN)

# This function measures the distance to the walls in front of the micromouse using the ultrasonic sensor
    def measure_distance_front(self):

# This initially sets the Trig pin to 0 to make sure that it is off and then sends the pulse. It measures the amount of time it takes. It must be divided by 2 because it has to travel there and back and you divide by 29.1 because of the speed of sound. Also multiply by 10 to get the distance in mm to keep consistency in units.   
        self.TRIG.value(0)
        time.sleep_us(2)
        self.TRIG.value(1)
        time.sleep_us(10)
        self.TRIG.value(0)
        duration = time_pulse_us(self.ECHO, 1, timeout_us=30000)  

# This is for when it takes longer than the timeout amount i.e the wall is not within a specific distance. This will be changed but I will need to calculate the timeout in terms of the threshold distance that is a placeholder in the previous constants (config.py) code 
        if duration == -2:
            return None 
        else:
            distance_mm = (duration / 2) * 0.343
            return distance_mm

# These functions get the distance from the left and right ToF sensors    
    def get_left_distance(self):
        try:
            return self.tof_left.range()
        except Exception:
            print("Error reading left ToF sensor")
            return None
    
    def get_right_distance(self):
        try:
            return self.tof_right.range()
        except Exception:
            print("Error reading right ToF sensor")
            return None
