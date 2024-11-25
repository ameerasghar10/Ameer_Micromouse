import time
from machine import Pin, PWM

# Delay execution by 5 seconds to allow time for setup
time.sleep(5)

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
encoder_left_A = Pin(10, Pin.IN)   # GP10 for left encoder A
encoder_left_B = Pin(11, Pin.IN)   # GP11 for left encoder B
encoder_right_A = Pin(12, Pin.IN)  # GP12 for right encoder A
encoder_right_B = Pin(13, Pin.IN)  # GP13 for right encoder B

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

# Function to move forward a specific distance while keeping straight
def move_forward_distance(distance_mm, Kp=1000):
    global count_left, count_right
    # Calculate target counts
    wheel_circumference = 3.1416 * 40  # π * diameter in mm
    counts_per_rev = 12 * 20  # Encoder counts per wheel revolution
    rotations_needed = distance_mm / wheel_circumference
    target_counts = int(rotations_needed * counts_per_rev)

    # Reset encoder counts
    count_left = 0
    count_right = 0

    # Set motors to move forward
    direction_left = 1   # Left motor moves forward with direction 1
    direction_right = 0  # Right motor moves forward with direction 0

    max_speed = 65025  # Maximum speed value for PWM
    min_speed = 20000  # Minimum speed to keep the motors moving

    # Base speeds for each motor
    base_speed_left = 30000  # Adjust as needed
    base_speed_right = 30000  # Adjust as needed

    # Start moving
    set_motor('left', direction_left, base_speed_left)
    set_motor('right', direction_right, base_speed_right)

    try:
        while True:
            # Calculate absolute counts for comparison
            abs_count_left = abs(count_left)
            abs_count_right = abs(count_right)

            # Check if target counts reached
            if abs_count_left >= target_counts and abs_count_right >= target_counts:
                break

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

            # Optionally, print counts and speeds for debugging
            print(f"Left Count: {count_left}, Right Count: {count_right}, Speed Left: {speed_left}, Speed Right: {speed_right}")

            # Short delay to prevent high CPU usage
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors after movement
        stop_motors()
        # Print final counts
        print(f"Final Left Count: {count_left}, Final Right Count: {count_right}")

# Main code to move forward 180 cm (adjust distance_mm as needed)
move_forward_distance(180)  # 180 mm is 18 cm
