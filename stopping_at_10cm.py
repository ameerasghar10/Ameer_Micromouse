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
counts_per_rev = (12 * 20)/2  

rotations_per_cell = cell_length_mm / wheel_circumference
counts_per_cell = rotations_per_cell * counts_per_rev

cells_moved = 0
counts_since_last_cell = 0  

prev_abs_count_left = 0
prev_abs_count_right = 0

set_motor('left', direction_left, base_speed_left)
set_motor('right', direction_right, base_speed_right)

# All of the code above is the same initialisation as the code from the move_one_cell_test.py code
try:
    while True:
# This reads the distances from each ToF sensor 
        distance_front = sensor_front.range()
        distance_left = sensor_left.range()
        distance_right = sensor_right.range()

        print("Front Distance:", distance_front, "mm")
        print("Left Distance:", distance_left, "mm")
        print("Right Distance:", distance_right, "mm")

        stop_distance = 200  
        if distance_front <= stop_distance and distance_front != 0:
            print(f"Maze wall detected at {distance_front} mm. Stopping.")
            stop_motors()
            break
        
        abs_count_left = abs(count_left)
        abs_count_right = abs(count_right)
       
        delta_count_left = abs_count_left - prev_abs_count_left
        delta_count_right = abs_count_right - prev_abs_count_right
       
        counts_since_last_cell += (delta_count_left + delta_count_right) / 2  # Average delta counts
        
        prev_abs_count_left = abs_count_left
        prev_abs_count_right = abs_count_right

# This makes sure that when the number of delta counts have totalled to make the counts since the lasy cell equal to the number of counts that it needs to move one cell, then the number of counts per cell is removed from the counts since the last cell so that the loop can be repeated
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