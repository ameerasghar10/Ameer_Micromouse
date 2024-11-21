# These are the pins for setting the direction and speeds of the two motors and also the PWM frequency
MOTOR_A_DIR_PIN = 6
MOTOR_A_PWM_PIN = 7
MOTOR_B_DIR_PIN = 8
MOTOR_B_PWM_PIN = 9
MOTOR_PWM_FREQ = 1000 

# This defines the pins for the encoders and the number of encoder counts per revolution is a placeholder
ENCODER_A_PIN_A = 10
ENCODER_A_PIN_B = 11
ENCODER_B_PIN_A = 12
ENCODER_B_PIN_B = 13


# These are the pins used for the ToF sensors and the Ultrasonic sensor 
TOF_LEFT_SDA = 0
TOF_LEFT_SCL = 1
TOF_RIGHT_SDA = 2
TOF_RIGHT_SCL = 3

ULTRASONIC_TRIG_PIN = 14
ULTRASONIC_ECHO_PIN = 15

# These ahave been updated now for certain constant values
WHEEL_DIAMETER_MM = 40
ENCODER_COUNTS_PER_REV = 12
GEAR_RATIO = 20
EFFECTIVE_CPR = ENCODER_COUNTS_PER_REV * GEAR_RATIO
WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * 3.14159265
DISTANCE_PER_COUNT_MM = WHEEL_CIRCUMFERENCE_MM / EFFECTIVE_CPR

MAZE_CELL_SIZE_MM = 180
COUNTS_PER_CELL = int((MAZE_CELL_SIZE_MM / DISTANCE_PER_COUNT_MM) + 0.5)

# These are placeholders for what we decide to be certain values depending on how the micromouse functions and can be iterated
BASE_SPEED = 32768  
SPEED_CORRECTION = 1000  
SIDE_DISTANCE_THRESHOLD_MM = 5  

# This defines the initial position and begins with North but can be changed depending on the maze. If starting with an unknown maze, a function may be written to work out where the micromouse is in the maze but for this project we will know whaat the maze is
INITIAL_POSITION = (0, 0) 
INITIAL_DIRECTION = 'N'  
