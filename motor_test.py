from machine import Pin, PWM
import time


dir1 = Pin(6, Pin.OUT)
pwm1 = PWM(Pin(7))
pwm1.freq(1000)  


dir2 = Pin(8, Pin.OUT)
pwm2 = PWM(Pin(9))
pwm2.freq(1000)  


def set_motor(motor, direction, speed):
    if motor == 'A':
        dir1.value(direction)
        pwm1.duty_u16(speed)  
    elif motor == 'B':
        dir2.value(direction)
        pwm2.duty_u16(speed)  


try:
    while True:
        
        set_motor('A', 1, 30000)
        set_motor('B', 1, 30000)
        time.sleep(2)
        
        set_motor('A', 1, 0)
        set_motor('B', 1, 0)
        time.sleep(1)
       
        set_motor('A', 0, 30000)
        set_motor('B', 0, 30000)
        time.sleep(2)
        
        set_motor('A', 0, 0)
        set_motor('B', 0, 0)
        time.sleep(1)
except KeyboardInterrupt:
    
    set_motor('A', 1, 0)
    set_motor('B', 1, 0)
