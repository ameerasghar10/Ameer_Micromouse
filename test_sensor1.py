from machine import I2C, Pin
from vl6180x_sensor import Sensor  
import time

i2c0 = I2C(0, scl=Pin(1), sda=Pin(0))  


sensor1 = Sensor(i2c0)


while True:
    try:
        distance = sensor1.range()  
        print("Sensor 1 Distance:", distance, "mm")
        time.sleep(1)  
    except Exception as e:
        print("Error with Sensor 1:", e)
        break
