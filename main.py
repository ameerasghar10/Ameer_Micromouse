import time
from machine import I2C, Pin, SoftI2C, PWM
from vl6180x_sensor import Sensor

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

wheel_diameter_mm = 40
wheel_circumference = 3.1416 * wheel_diameter_mm
counts_per_rev = 120
cell_length_mm = 170

Kp = 1000

max_speed = 65025
min_speed = 20000
base_speed_left = 30000
base_speed_right = 30000
direction_left = 1
direction_right = 0
wall_threshold = 75
robot_width_mm = 80
turn_calibration_factor = 0.8

def move_forward_distance(distance_mm):
    global count_left, count_right
    rotations_needed = distance_mm / wheel_circumference
    target_counts = int(rotations_needed * counts_per_rev)
    count_left = 0
    count_right = 0
    dir_left_val = 1
    dir_right_val = 0

    offset = 2500

    set_motor('left', dir_left_val, base_speed_left)
    set_motor('right', dir_right_val, base_speed_right)
    while True:
        abs_left = abs(count_left)
        abs_right = abs(count_right)

        if abs_left >= target_counts and abs_right >= target_counts:
            break

        error = abs_left - abs_right
        correction = int(Kp * error)

        speed_left = base_speed_left - correction
        speed_right = base_speed_right + correction + offset

        speed_left = min(max(speed_left, min_speed), max_speed)
        speed_right = min(max(speed_right, min_speed), max_speed)

        set_motor('left', dir_left_val, speed_left)
        set_motor('right', dir_right_val, speed_right)
        time.sleep(0.01)

    stop_motors()
    time.sleep(0.2)

def turn_left_90():
    global count_left, count_right
    count_left = 0
    count_right = 0
    turn_counts = int((robot_width_mm * counts_per_rev * turn_calibration_factor) / (4 * wheel_diameter_mm))
    set_motor('left', 0, base_speed_left)
    set_motor('right', direction_right, base_speed_right)
    while True:
        if abs(count_left) >= turn_counts and abs(count_right) >= turn_counts:
            break
        time.sleep(0.01)
    stop_motors()
    time.sleep(0.2)

def turn_right_90():
    global count_left, count_right
    count_left = 0
    count_right = 0
    turn_counts = int((robot_width_mm * counts_per_rev * turn_calibration_factor) / (4 * wheel_diameter_mm))
    set_motor('left', direction_left, base_speed_left)
    set_motor('right', 1, base_speed_right)
    while True:
        if abs(count_left) >= turn_counts and abs(count_right) >= turn_counts:
            break
        time.sleep(0.01)
    stop_motors()
    time.sleep(0.2)

def turn_180():
    turn_left_90()
    turn_left_90()

while True:
    move_forward_distance(cell_length_mm)
    stop_motors()
    time.sleep(0.2)

    dist_left = sensor_left.range()
    dist_front = sensor_front.range()
    dist_right = sensor_right.range()

    no_left_wall = dist_left > wall_threshold
    no_front_wall = dist_front > wall_threshold
    no_right_wall = dist_right > wall_threshold

    if no_left_wall:
        turn_left_90()
    elif no_front_wall:
        pass
    elif no_right_wall:
        turn_right_90()
    else:
        turn_180()
