from machine import I2C, Pin, SoftI2C, PWM
from vl6180x_sensor import Sensor
import time

time.sleep(5)

i2c0 = I2C(0, scl=Pin(1), sda=Pin(0))   
i2c1 = I2C(1, scl=Pin(3), sda=Pin(2))   
i2c2 = SoftI2C(scl=Pin(5), sda=Pin(4))  

sensor_front = Sensor(i2c0)
sensor_left = Sensor(i2c1)
sensor_right = Sensor(i2c2)

dir_left = Pin(6, Pin.OUT)    
pwm_left = PWM(Pin(7))        
pwm_left.freq(1000)

dir_right = Pin(8, Pin.OUT)   
pwm_right = PWM(Pin(9))       
pwm_right.freq(1000)

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

encoder_left_A = Pin(10, Pin.IN)
encoder_left_B = Pin(11, Pin.IN)
encoder_right_A = Pin(12, Pin.IN)
encoder_right_B = Pin(13, Pin.IN)

count_left = 0
count_right = 0

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

encoder_left_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_left_callback)
encoder_right_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_right_callback)

direction_left = 1   
direction_right = 0  


max_speed = 65025     
min_speed = 20000     
base_speed_left = 30000  
base_speed_right = 30000  
Kp = 1000 

cell_length_mm = 180  
wheel_diameter_mm = 40  
wheel_circumference = 3.1416 * wheel_diameter_mm  
counts_per_rev = 120

rotations_per_cell = cell_length_mm / wheel_circumference
counts_per_cell = rotations_per_cell * counts_per_rev

cells_moved = 0
counts_since_last_cell = 0 

prev_abs_count_left = 0
prev_abs_count_right = 0

# The distance between the wheels
robot_width_mm = 80
# Just tested until it worked
turn_calibration_factor = 0.5

def turn_left_90():
    global count_left, count_right

    count_left = 0
    count_right = 0

# This is the calculation for how many counts are needed for the micrmouse to turn 90 degrees to the left 
    turn_counts = int((robot_width_mm * counts_per_rev * turn_calibration_factor) / (4 * wheel_diameter_mm))

    print(f"Turn counts needed for 90-degree turn: {turn_counts}")

# To turn in a still position, the wheels need to be moving in opposite directions. Of course for this code oppoiste directions means that they have the same value due to the the motors being in opooiste directions 
    set_motor('left', 0, base_speed_left)  
    set_motor('right', direction_right, base_speed_right)  

    while True:        
        abs_count_left = abs(count_left)
        abs_count_right = abs(count_right)
        
        print(f"Turning - Left Count: {abs_count_left}, Right Count: {abs_count_right}")
       
        if abs_count_left >= turn_counts and abs_count_right >= turn_counts:
            break

        time.sleep(0.01)

# This stops the motors after the micromouse has turned 90 degrees to the left 
    stop_motors()
    print("Turn completed.")

set_motor('left', direction_left, base_speed_left)
set_motor('right', direction_right, base_speed_right)

try:
    while True:
        distance_front = sensor_front.range()
        distance_left = sensor_left.range()
        distance_right = sensor_right.range()

        print("Front Distance:", distance_front, "mm")
        print("Left Distance:", distance_left, "mm")
        print("Right Distance:", distance_right, "mm")

        stop_distance = 100 
        if distance_front <= stop_distance and distance_front != 0:
            print(f"Obstacle detected at {distance_front} mm. Turning left.")
            stop_motors()
            time.sleep(0.5)
            
            turn_left_90()
            break 
        
        abs_count_left = abs(count_left)
        abs_count_right = abs(count_right)

        delta_count_left = abs_count_left - prev_abs_count_left
        delta_count_right = abs_count_right - prev_abs_count_right

        counts_since_last_cell += (delta_count_left + delta_count_right) / 2 

        prev_abs_count_left = abs_count_left
        prev_abs_count_right = abs_count_right

        if counts_since_last_cell >= counts_per_cell:
            cells_moved += 1
            counts_since_last_cell -= counts_per_cell  
            print(f"Cells moved: {cells_moved}")
        
        error = abs_count_left - abs_count_right
        correction = int(Kp * error)

        speed_left = base_speed_left - correction
        speed_right = base_speed_right + correction

        speed_left = min(max(speed_left, min_speed), max_speed)
        speed_right = min(max(speed_right, min_speed), max_speed)

        set_motor('left', direction_left, speed_left)
        set_motor('right', direction_right, speed_right)

        print(f"Left Count: {count_left}, Right Count: {count_right}, "
              f"Speed Left: {speed_left}, Speed Right: {speed_right}")

        time.sleep(0.01)

except KeyboardInterrupt:
    pass
finally:    
    stop_motors()
    print(f"Final Left Count: {count_left}, Final Right Count: {count_right}")
    print(f"Total cells moved: {cells_moved}")
