from controller import Robot, Motor, DistanceSensor, Camera
import numpy as np
import math
import cv2

# Constants
MAX_SPEED = 5.0  
TURN_TIME=6.9/MAX_SPEED
# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize distance sensors
dist_sensors = {
    "left": robot.getDevice("ds_left"),
    "right": robot.getDevice("ds_right"),
    "front": robot.getDevice("ds_center"),
    "left_90": robot.getDevice("ds_left_90"),
    "right_90": robot.getDevice("ds_right_90")
}
for sensor in dist_sensors.values():
    sensor.enable(timestep)

# Initialize camera
camera = robot.getDevice("camera")
camera.enable(timestep)

# Initialize motors
motors = [
    robot.getDevice("wheel1"),
    robot.getDevice("wheel2"),
    robot.getDevice("wheel3"),
    robot.getDevice("wheel4")
]
for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

# Position and orientation variables
x, y, orientation = 0, 0, 0  # Starting position (0, 0), facing upwards (0 degrees)

# Movement functions
def move_forward():
    for motor in motors:
        motor.setVelocity(MAX_SPEED)
    update_position('forward', MAX_SPEED)  # Update position after moving forward

def stop_motors():
    for motor in motors:
        motor.setVelocity(0)

def turn_left():
    motors[0].setVelocity(-MAX_SPEED)
    motors[1].setVelocity(MAX_SPEED)
    motors[2].setVelocity(-MAX_SPEED)
    motors[3].setVelocity(MAX_SPEED)
    
    elapsed_time = 0
    while elapsed_time < TURN_TIME:
        robot.step(timestep)
        elapsed_time += timestep / 1000.0
    stop_motors()
    update_position('turn_left', 0)  # Update position after turning left

def turn_right():
    motors[0].setVelocity(MAX_SPEED)
    motors[1].setVelocity(-MAX_SPEED)
    motors[2].setVelocity(MAX_SPEED)
    motors[3].setVelocity(-MAX_SPEED)
    
    elapsed_time = 0
    while elapsed_time < TURN_TIME:
        robot.step(timestep)
        elapsed_time += timestep / 1000.0
    stop_motors()
    update_position('turn_right', 0)  # Update position after turning right

def turn_180():
    motors[0].setVelocity(MAX_SPEED)
    motors[1].setVelocity(-MAX_SPEED)
    motors[2].setVelocity(MAX_SPEED)
    motors[3].setVelocity(-MAX_SPEED)
    
    # Assuming TURN_TIME is the time for a 90-degree turn, we double it for 180 degrees
    elapsed_time = 0
    turn_duration = 1.3601*2
    while elapsed_time < turn_duration:
        robot.step(timestep)
        elapsed_time += timestep / 1000.0
    
    stop_motors()
    update_position('turn_left', 0)  # Update position after turning 180 degrees

# Position Tracking Logic
def get_position():
    """
    Get the current position of the robot based on movement history.
    Returns a tuple (x, y, orientation).
    """
    return (x, y, orientation)

def update_position(movement, distance):
    """
    Update the robot's position based on its movement (forward, turn) and distance.
    movement: 'forward', 'turn_left', 'turn_right'
    distance: the distance moved (or angle for turns)
    """
    global x, y, orientation

    if movement == 'forward':
        # Move forward by 'distance' units
        x += distance * math.cos(math.radians(orientation))
        y += distance * math.sin(math.radians(orientation))

    elif movement == 'turn_left':
        # Turn left by 90 degrees
        orientation = (orientation + 90) % 360

    elif movement == 'turn_right':
        # Turn right by 90 degrees
        orientation = (orientation - 90) % 360

    elif movement == 'turn_180':
        # Turn 180 degrees
        orientation = (orientation - 180) % 360

# Movement logging
movement_log = []
movement_turn_log = []

# Color detection functions
def get_color_name(h, s, v):
    """Determine the color name from HSV values."""
    if 60 <= h < 90:
        return "Green"
    else:
        return "Unknown"

def detect_color():
    """Detects the dominant color in the camera frame."""
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    
    # Get the center pixel color
    center_x, center_y = width // 2, height // 2
    r = Camera.imageGetRed(image, width, center_x, center_y)
    g = Camera.imageGetGreen(image, width, center_x, center_y)
    b = Camera.imageGetBlue(image, width, center_x, center_y)
    
    # Convert RGB to HSV
    rgb_pixel = np.uint8([[[r, g, b]]])
    hsv_pixel = cv2.cvtColor(rgb_pixel, cv2.COLOR_RGB2HSV)[0][0]
    h, s, v = hsv_pixel

    # Get color name
    color_name = get_color_name(h, s, v)

    return color_name

# Maze navigation logic
def navigate_maze():
    iteration = 0  
    rescued = 1
    while robot.step(timestep) != -1 and iteration < 1000:
        front_distance = dist_sensors["front"].getValue()
        right_distance = dist_sensors["right"].getValue()
        left_distance = dist_sensors["left"].getValue()
        right_90_distance = dist_sensors["right_90"].getValue()
        left_90_distance = dist_sensors["left_90"].getValue()
        
        # Robot movement logic
        if front_distance < 560 and detect_color() == "Green":
            stop_motors()
            rescued = rescued + 1
            print(f"{rescued} survivors detected")
            robot.step(3000)
            print(f"{rescued} survivors rescued")
            turn_180()
            if rescued == 3:
                return_to_start()
                break  # Exit loop if trapped
        elif front_distance > 540:
            move_forward()
            movement_log.append(("forward", MAX_SPEED))
        elif left_distance < 1000 and right_90_distance == 1000:
             if (movement_turn_log[-1]=="right" and movement_turn_log[-2]=="right" and movement_turn_log[-3]=="right" and movement_turn_log[-4]=="right") :
                turn_left()
                movement_log.append(("left", 0))
                movement_turn_log.append("left")
             else:
                turn_right()
                movement_log.append(("right", 0))
                movement_turn_log.append("right")
        elif right_distance < 1000 and left_90_distance == 1000:
            turn_left()
            movement_log.append(("left", 0))
            movement_turn_log.append("left")
        elif right_distance == 1000 and right_90_distance < 1000 and left_90_distance == 1000:
            turn_right()
            movement_log.append(("right", 0))
            movement_turn_log.append("right")
        elif left_distance == 1000 and left_90_distance < 1000 and right_90_distance == 1000:
            turn_left()
            movement_log.append(("left", 0))
            movement_turn_log.append("left")
        elif left_90_distance < 1000 and right_90_distance < 1000 and left_distance < 1000 and right_distance < 1000:
            turn_right()
            movement_log.append(("right", 0))
            movement_turn_log.append("right")
        elif left_90_distance == 1000 and right_90_distance == 1000:
            turn_left()
            movement_log.append(("left", 0))
            movement_turn_log.append("left")
        else:
            stop_motors()
            movement_log.append(("stop", 0))
            break  # Exit loop if trapped

# Backtracking logic using stored positions
def return_to_start():
    """Backtrack using stored movements and positions."""
    global x, y, orientation  # Use global variables x, y, orientation
    print("Returning to start position...")

    # Backtrack through the movement log and return to the start
    for movement, distance in reversed(movement_log):
        robot.step(30)
        if movement == "forward":
            # Move back by the opposite distance
            x -= distance * math.cos(math.radians(orientation))
            y -= distance * math.sin(math.radians(orientation))
        elif movement == "left":
            orientation = (orientation + 90) % 360
        elif movement == "right":
            orientation = (orientation - 90) % 360
        elif movement == "stop":
            break

    print(f"Returned to start position: {x}, {y}, Orientation: {orientation}")
navigate_maze()