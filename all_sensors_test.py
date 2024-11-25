from machine import I2C, Pin, SoftI2C
from vl6180x_sensor import Sensor  
import time 

i2c0 = I2C(0, scl=Pin(1), sda=Pin(0))  
i2c1 = I2C(1, scl=Pin(3), sda=Pin(2))
i2c3 = SoftI2C(scl=Pin(5), sda=Pin(4))

sensor1 = Sensor(i2c0)
sensor2 = Sensor(i2c1)
sensor3 = Sensor(i2c3)


while True:
        distance1 = sensor1.range()
        distance2 = sensor2.range()
        distance3 = sensor3.range()
        print("Sensor 1 Distance:", distance1, "mm")
        print("Sensor 2 Distance:", distance2, "mm")
        print("Sensor 3 Distance:", distance3, "mm")
        time.sleep(1)

    
