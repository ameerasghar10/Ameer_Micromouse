from machine import Pin, time_pulse_us
import time


TRIG = Pin(14, Pin.OUT)  
ECHO = Pin(15, Pin.IN)   


def measure_distance():
   
    TRIG.value(0)
    time.sleep_us(2)

  
    TRIG.value(1)
    time.sleep_us(10)
    TRIG.value(0)

    
    duration = time_pulse_us(ECHO, 1)

    
    distance = (duration / 2) / 29.1  
    return distance


while True:
    dist = measure_distance()
    print("Distance:", dist, "cm")
    time.sleep(1)  
