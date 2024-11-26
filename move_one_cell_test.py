import time
from machine import I2C, Pin, SoftI2C, PWM

# 5 second delay so that we can place the micromouse into the maze before it starts after switching on the battery
time.sleep(5)

# Front
i2c0 = I2C(0, scl=Pin(1), sda=Pin(0))
# Left
i2c1 = I2C(1, scl=Pin(3), sda=Pin(2))   
# Right
i2c2 = SoftI2C(scl=Pin(5), sda=Pin(4))  

from vl6180x_sensor import Sensor
sensor_front = Sensor(i2c0)
sensor_left = Sensor(i2c1)
sensor_right = Sensor(i2c2)

# Left Motor
dir_left = Pin(6, Pin.OUT)    
pwm_left = PWM(Pin(7))        
pwm_left.freq(1000)
# Right Motor
dir_right = Pin(8, Pin.OUT)   
pwm_right = PWM(Pin(9))       
pwm_right.freq(1000)

# Because the motors are opposite eachother they are in different directions so after testing 0 is actually forward for the right motor
def set_motor(motor, direction, speed):
    if motor == 'left':
        dir_left.value(direction)
        pwm_left.duty_u16(speed)
    elif motor == 'right':
        dir_right.value(direction)
        pwm_right.duty_u16(speed)


def stop_motors():
    set_motor('left', dir_left.value(), 0)
    set_motor('right', dir_right.value(), 0)

encoder_left_A = Pin(10, Pin.IN)   # GP10 for left encoder A
encoder_left_B = Pin(11, Pin.IN)   # GP11 for left encoder B
encoder_right_A = Pin(12, Pin.IN)  # GP12 for right encoder A
encoder_right_B = Pin(13, Pin.IN)  # GP13 for right encoder B

count_left = 0
count_right = 0

# This is for updating the encoders' counts
def encoder_left_callback(pin):
    global count_left
    if encoder_left_A.value() == encoder_left_B.value():
        count_left += 1
    else:
        count_left -= 1

def encoder_right_callback(pin):
    global count_right
    if encoder_right_A.value() == encoder_right_B.value():
        count_right += 1
    else:
        count_right -= 1

# Keeps updating the encouder counts 
encoder_left_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_left_callback)
encoder_right_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_right_callback)

# A function to move forward a specific distance while keeping the micromouse straight
def move_forward_distance(distance_mm, Kp=500):
    global count_left, count_right
    wheel_diameter_mm = 40  
    wheel_circumference = 3.1416 * wheel_diameter_mm  
    counts_per_rev = 12 * 20 / 2  
    distance_target_mm = distance_mm  

    
    rotations_needed = distance_target_mm / wheel_circumference
    target_counts = int(rotations_needed * counts_per_rev)

    print(f"Rotations needed: {rotations_needed:.2f}")
    print(f"Target counts: {target_counts}")

   
    count_left = 0
    count_right = 0

    
    direction_left = 1   
    direction_right = 0  

    max_speed = 65025  
    min_speed = 20000  

    
    base_speed_left = 30000  
    base_speed_right = 30000  

    
    set_motor('left', direction_left, base_speed_left)
    set_motor('right', direction_right, base_speed_right)

    try:
        while True:
            # Due to the way that the motors are placed one will give negative values and one positive when moving forward and so it's better to compare absolute values. Also better to use absolute than to manually correct in the function as it reminds me that one is negative.              
            abs_count_left = abs(count_left)
            abs_count_right = abs(count_right)

            # This helps me check the actual encoder counts for testing especially when adjusting Kp           
            print(f"Left Count: {count_left}, Right Count: {count_right}")

            # This checks if both encoders have reached the number of ticks that they need to travel the distance I decide and if they both have ('and' statement) then it stops moving              
            if abs_count_left >= target_counts and abs_count_right >= target_counts:
                print("Target distance reached. Stopping motors.")
                break

            # This caclulates the difference between the absolute values of the right and left encoder     
            error = abs_count_left - abs_count_right
            correction = int(Kp * error)

            
            speed_left = base_speed_left - correction
            speed_right = base_speed_right + correction

            # This makes sure that the correction doesn't make the micromouse speed too slow or too fast            
            speed_left = min(max(speed_left, min_speed), max_speed)
            speed_right = min(max(speed_right, min_speed), max_speed)

            set_motor('left', direction_left, speed_left)
            set_motor('right', direction_right, speed_right)

            
            print(f"Adjusted Speeds are Left: {speed_left}, Right: {speed_right}")

            # I read online that you need a timeout
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Something stopped the micromouse running, please check for mistakes.")
    finally:
        stop_motors()
        print(f"Final Left Count: {count_left}, Final Right Count: {count_right}")

# Code to move forward 200mm or whatever we choose
move_forward_distance(200)  # 200 mm is 20 cm

