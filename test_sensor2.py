from machine import I2C, Pin
from vl6180x_sensor import Sensor  
import time  


i2c1 = I2C(1, scl=Pin(3), sda=Pin(2)) 


sensor2 = Sensor(i2c1)


while True:
    try:
        distance = sensor2.range()  
        print("Sensor 2 Distance:", distance, "mm")
        time.sleep(1)  
    except Exception as e:
        print("Error with Sensor 2:", e)
        break
