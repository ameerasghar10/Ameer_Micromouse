from machine import Pin, PWM
from config import MOTOR_A_DIR_PIN, MOTOR_A_PWM_PIN, MOTOR_B_DIR_PIN, MOTOR_B_PWM_PIN, MOTOR_PWM_FREQ

class MotorControl:
    def __init__(self):
        
        # This defines the pin that will control the motor (A) and the PWM pin to control its speed
        self.dir1 = Pin(MOTOR_A_DIR_PIN, Pin.OUT)
        self.pwm1 = PWM(Pin(MOTOR_A_PWM_PIN))
        self.pwm1.freq(MOTOR_PWM_FREQ)
        
        # This is the same as for motor (A) above but for motor (B) instead
        self.dir2 = Pin(MOTOR_B_DIR_PIN, Pin.OUT)
        self.pwm2 = PWM(Pin(MOTOR_B_PWM_PIN))
        self.pwm2.freq(MOTOR_PWM_FREQ)

# This is the actual function that controls the motors. Direction will replaced by 1 or 0 for forward or backwards and speed will be replaced by a value between the minimum of 0 to the maximum of 65535    
    def set_motor(self, motor, direction, speed):
        if motor == 'A':
            self.dir1.value(direction)
            self.pwm1.duty_u16(speed)
        elif motor == 'B':
            self.dir2.value(direction)
            self.pwm2.duty_u16(speed)
    
# I arbitrarily chose a direction but it doesn't really matter when stopping what direction you choose (the stopping is immediate)
    def stop_motors(self):
        self.set_motor('A', 1, 0)
        self.set_motor('B', 1, 0)
