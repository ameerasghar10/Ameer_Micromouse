import time
from config import *
from motor_control import MotorControl
from encoders import Encoders
from sensors import Sensors
from position_tracking import PositionTracker

motor_control = MotorControl()
encoders = Encoders()
sensors = Sensors()
position_tracker = PositionTracker(INITIAL_POSITION, INITIAL_DIRECTION)

# This a function that will move the micromouse forward by one square(cell) in the maze. First it initialises all the neccessary parts for the function.
def move_one_cell():
    
# This sets the encoder counts to 0 so that if we're beginning to move from a place it starts at 0 and not another number
    encoders.reset_counts()
    
# This sets both motors to begin moving forward at the speed which we defined as roughly half of the maximum speed
    motor_control.set_motor('A', 1, BASE_SPEED)
    motor_control.set_motor('B', 1, BASE_SPEED)

# This loop checks the left and right encoder counts and uses an average calculation to see how far the micromouse has actually moved
    try:
        while True:
            left_count, right_count = encoders.get_counts()
            avg_count = (left_count + right_count) / 2
            
# This uses the ultrasonic sensor to check what the distance is to any walls that might be in front and if it is within the threshold limit that will be determined in the constants. If there is the loop is stopped by the break. 
            dist_front = sensors.measure_distance_front()
            if dist_front is not None and dist_front < FRONT_DISTANCE_THRESHOLD_MM:
                print("Wall detected ahead")
                break
            
# This checks if the micromouse has moved the distance that it needs to move to get from one cell to the next cell. Once the distance is equal it will also break and stop moving.
            if avg_count >= COUNTS_PER_CELL:
                print("Moved one cell forward")
                break
            
# This gets the distance values from the ToF sensors
            left_distance = sensors.get_left_distance()
            right_distance = sensors.get_right_distance()
            
# This changes the motor speeds. If the encoder left count is higher then the right count then it means that the left wheel is moving faster than the right wheel so it slows down and vice versa to keep the micromouse moving straight 
            if left_count > right_count:
                
                motor_control.set_motor('A', 1, BASE_SPEED - SPEED_CORRECTION)
                motor_control.set_motor('B', 1, BASE_SPEED + SPEED_CORRECTION)
            elif right_count > left_count:
                
                motor_control.set_motor('A', 1, BASE_SPEED + SPEED_CORRECTION)
                motor_control.set_motor('B', 1, BASE_SPEED - SPEED_CORRECTION)
            else:
                
                motor_control.set_motor('A', 1, BASE_SPEED)
                motor_control.set_motor('B', 1, BASE_SPEED)
            
# This also changes the motor speeds but based on the ToF sensors so that the micromouse stays in the centre of the cell or in a straight line. If it is too close to the left wall it will speed up the left wheel and slow down the right and vice versa.
            if left_distance is not None and right_distance is not None:
                if abs(left_distance - right_distance) > SIDE_DISTANCE_THRESHOLD_MM:
                    if left_distance < right_distance:
                       
                        motor_control.set_motor('A', 1, BASE_SPEED + SPEED_CORRECTION)
                        motor_control.set_motor('B', 1, BASE_SPEED - SPEED_CORRECTION)
                    elif right_distance < left_distance:
                        
                        motor_control.set_motor('A', 1, BASE_SPEED - SPEED_CORRECTION)
                        motor_control.set_motor('B', 1, BASE_SPEED + SPEED_CORRECTION)

# According to research I read online it can be useful to add a delay to minimise the risk of overheating and to give the sensors time to provide new values            
            time.sleep(0.01)  
    
    except KeyboardInterrupt:
        pass
    
# Stops the motors
    motor_control.stop_motors()
    
# This updates the micromouse's position after moving one cell and prints the new current position and direction it's facing
    position_tracker.update_position('forward')
    current_position = position_tracker.get_position()
    print(f"Current position: {current_position}")
    print(f"Current direction: {position_tracker.get_direction()}")

if __name__ == "__main__":
    move_one_cell()
