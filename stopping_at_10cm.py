from machine import I2C, Pin, SoftI2C, PWM
from vl6180x_sensor import Sensor
import time

# Delay execution by 5 seconds to allow time for setup
time.sleep(5)

# Initialize I2C buses for sensors
i2c0 = I2C(0, scl=Pin(1), sda=Pin(0))   # Front sensor
i2c1 = I2C(1, scl=Pin(3), sda=Pin(2))   # Left sensor
i2c2 = SoftI2C(scl=Pin(5), sda=Pin(4))  # Right sensor

# Initialize sensors
sensor_front = Sensor(i2c0)
sensor_left = Sensor(i2c1)
sensor_right = Sensor(i2c2)

# Motor control pins
dir_left = Pin(6, Pin.OUT)    # GP6 for left motor direction
pwm_left = PWM(Pin(7))        # GP7 for left motor PWM
pwm_left.freq(1000)

dir_right = Pin(8, Pin.OUT)   # GP8 for right motor direction
pwm_right = PWM(Pin(9))       # GP9 for right motor PWM
pwm_right.freq(1000)

# Function to control motors
def set_motor(motor, direction, speed):
    if motor == 'left':
        dir_left.value(direction)
        pwm_left.duty_u16(speed)
    elif motor == 'right':
        dir_right.value(direction)
        pwm_right.duty_u16(speed)

# Function to stop motors
def stop_motors():
    set_motor('left', dir_left.value(), 0)
    set_motor('right', dir_right.value(), 0)

# Encoder pins
encoder_left_A = Pin(10, Pin.IN)
encoder_left_B = Pin(11, Pin.IN)
encoder_right_A = Pin(12, Pin.IN)
encoder_right_B = Pin(13, Pin.IN)

# Encoder counts
count_left = 0
count_right = 0

# Encoder callback functions
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

# Set up interrupts for encoders
encoder_left_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_left_callback)
encoder_right_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_right_callback)

# Initial motor directions
direction_left = 1   # Left motor moves forward with direction 1
direction_right = 0  # Right motor moves forward with direction 0

# Base speeds and control parameters
max_speed = 65025     # Maximum speed value for PWM
min_speed = 20000     # Minimum speed to keep the motors moving
base_speed_left = 30000  # Adjust as needed
base_speed_right = 30000  # Adjust as needed
Kp = 1000  # Proportional gain for speed correction

# Initialize cell tracking variables
cell_length_mm = 180  # Length of one cell in mm
wheel_diameter_mm = 40  # Wheel diameter in mm
wheel_circumference = 3.1416 * wheel_diameter_mm  # π * diameter
counts_per_rev = (12 * 20)/2  # Encoder counts per wheel revolution

# Calculate counts per cell
rotations_per_cell = cell_length_mm / wheel_circumference
counts_per_cell = rotations_per_cell * counts_per_rev

# Initialize cell counter and counts accumulator
cells_moved = 0
counts_since_last_cell = 0  # To track counts since last cell

# Initialize previous counts for delta calculation
prev_abs_count_left = 0
prev_abs_count_right = 0

# Start moving forward
set_motor('left', direction_left, base_speed_left)
set_motor('right', direction_right, base_speed_right)

try:
    while True:
        # Read distances from sensors
        distance_front = sensor_front.range()
        distance_left = sensor_left.range()
        distance_right = sensor_right.range()

        # Print sensor readings
        print("Front Distance:", distance_front, "mm")
        print("Left Distance:", distance_left, "mm")
        print("Right Distance:", distance_right, "mm")

        # Check for obstacle in front
        stop_distance = 200  # Distance threshold to stop (in mm)
        if distance_front <= stop_distance and distance_front != 0:
            print(f"Obstacle detected at {distance_front} mm. Stopping.")
            stop_motors()
            break

        # Calculate absolute counts for comparison
        abs_count_left = abs(count_left)
        abs_count_right = abs(count_right)

        # Calculate delta counts since last iteration
        delta_count_left = abs_count_left - prev_abs_count_left
        delta_count_right = abs_count_right - prev_abs_count_right

        # Update counts_since_last_cell with average of delta counts
        counts_since_last_cell += (delta_count_left + delta_count_right) / 2  # Average delta counts

        # Update previous counts for next iteration
        prev_abs_count_left = abs_count_left
        prev_abs_count_right = abs_count_right

        # Check if we have moved one cell
        if counts_since_last_cell >= counts_per_cell:
            cells_moved += 1
            counts_since_last_cell -= counts_per_cell  # Reset counts for next cell
            print(f"Cells moved: {cells_moved}")

        # Calculate error between left and right encoder counts
        error = abs_count_left - abs_count_right
        correction = int(Kp * error)

        # Adjust motor speeds
        speed_left = base_speed_left - correction
        speed_right = base_speed_right + correction

        # Ensure speeds are within limits
        speed_left = min(max(speed_left, min_speed), max_speed)
        speed_right = min(max(speed_right, min_speed), max_speed)

        set_motor('left', direction_left, speed_left)
        set_motor('right', direction_right, speed_right)

        # Print encoder counts and motor speeds for debugging
        print(f"Left Count: {count_left}, Right Count: {count_right}, "
              f"Speed Left: {speed_left}, Speed Right: {speed_right}")

        # Short delay to prevent high CPU usage
        time.sleep(0.01)

except KeyboardInterrupt:
    pass
finally:
    # Stop motors after movement
    stop_motors()
    # Print final counts and cells moved
    print(f"Final Left Count: {count_left}, Final Right Count: {count_right}")
    print(f"Total cells moved: {cells_moved}")