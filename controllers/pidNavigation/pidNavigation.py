"""my_controller controller."""

from controller import Robot
import math
import cv2
import numpy as np



# Create the Robot instance
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Function to get device instances
def get_device(device_name):
    device = robot.getDevice(device_name)
    if device:
        device.enable(timestep)
    return device


# Get ultrasonic sensors
us_names = ['ps4', 'ps2', 'ps0']  # Front, Left, Right
IR_sensors = [get_device(name) for name in us_names]

L_encoder = get_device('left encoder')
R_encoder = get_device('right encoder')

L_motor = robot.getDevice('left motor')
R_motor = robot.getDevice('right motor')

L_motor.setPosition(float('inf'))
R_motor.setPosition(float('inf'))

inertial_unit =get_device('Inertial Unit')
gyro = get_device('gyroScope')

camera=get_device('camera')

# Function to convert ultrasonic readings (depends on the LUT)
def ir_to_distance(value):
    return value * 0.01  # Adjust scaling based on your lookup table

def normalize_angle(angle):
    """Normalize angle to the range [-π, π]"""
    return math.atan2(math.sin(angle), math.cos(angle))

def angular_difference(target, current):
    """Compute shortest difference between two angles in radians."""
    diff = target - current
    return (diff + math.pi) % (2 * math.pi) - math.pi  # Keep in range [-π, π]

def use_camera(cam):
    image =cam.getImage()
    width = cam.getWidth()
    height = cam.getHeight()

    image_array = np.frombuffer(image,dtype=np.uint8).reshape((height,width,4))
    img_bgr = cv2.cvtColor(image_array,cv2.COLOR_RGB2BGR)
    img_rgb = image_array[:,:,:3]
    #print(img_rgb)


    new_height = int(height*2)
    new_width = int(width*2)
    img_resize = cv2.resize(img_rgb,(new_width,new_height),interpolation=cv2.INTER_LINEAR)

    return img_rgb
def moveForward():
    # PID for straight moves
    global stoping

    stoping = False
    robot.step(timestep)
    P=0.02
    I=0
    D=0.005
    last_error=0

    LstartVal = L_encoder.getValue()  # Start at 0 since we're subtracting the offsets
    start_time = robot.getTime()
    
    while robot.step(timestep) != -1:
        if axis=='X':
            left_read=IR_sensors[0].getValue()
            right_read=IR_sensors[2].getValue()
            # Get encoder values and subtract offset
        if (left_read > 900):
            error = 900 - left_read 
    
        elif (right_read > 900):
            error = right_read - 900
        else:
            error = 0
        print("Error:", error)
        correction = P * error+D*(error-last_error)
        last_error = error
        L_motor.setVelocity(2 - correction*0.01)
        R_motor.setVelocity(2 + correction*0.01)

        # Compute traveled distance
        LendVal = L_encoder.getValue()  # Since we already subtracted offset
        distance = (LendVal - LstartVal) * 0.06 * 3.14 / 7
        distance = round(distance, 3)
        
        
        end_time=robot.getTime()
        travel_time=end_time-start_time
        distance_T = 2*0.03*travel_time
        #print('Distance:', distance_T)
        #print('gyro',gyro.getValues())
        if(0.25<=distance_T<0.26):
            stoping =True
            print("stoping")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            robot.step(timestep*10)
            break

def turnRight():
    #turn 90 degree Right

    initial_Ryaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])  # Initial yaw
    target_angleR = normalize_angle(initial_Ryaw - math.radians(90))  # Target yaw (-90°)

    #print(f"Starting turn | Initial Yaw: {math.degrees(initial_Ryaw):.2f}° | Target Yaw: {math.degrees(target_angleR):.2f}°")

    L_motor.setVelocity(2)
    R_motor.setVelocity(-2)

    while robot.step(timestep) != -1:
        current_Ryaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])
        yaw_diffR = math.degrees(abs(angular_difference(target_angleR, current_Ryaw)))  # Fix wraparound

        #print(f"Yaw: {math.degrees(current_Ryaw):.2f}° | ΔYaw: {yaw_diffR:.2f}° ")

        if abs(yaw_diffR)<1:  # Stop when true yaw difference reaches 90°
            print("✅ Turn Complete! Stopping motors.")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            break

    robot.step(timestep * 10)

    
def turnLeft():
    #turn 90 degree Left
    initial_Lyaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])  # Extract yaw angle
   
    target_angleL= normalize_angle(initial_Lyaw+math.radians(90))  # Target yaw (-90 degrees)
    
    #print(f"Starting turn | Initial Yaw: {math.degrees(initial_Lyaw):.2f}° | Target Yaw: {math.degrees(target_angleL):.2f}°")
    
    L_motor.setVelocity(-2)
    R_motor.setVelocity(2)

    while(robot.step(timestep) != -1):
        cuurent_Lyaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])

        yaw_diffL = math.degrees(abs(angular_difference(target_angleL, cuurent_Lyaw))) 

        #print(f"Yaw: {math.degrees(cuurent_Lyaw):.2f}° | ΔYaw: {yaw_diffL:.2f}° ")
        
      
        if(abs(yaw_diffL)<1):
            print("stoping")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            break
    robot.step(timestep*10)
       
def turnReverse():
    turnRight()
    turnRight()

def moveBack():
    #move 0.25 meter backward with pid
    pass

def get_direction():
    
    pass
def get_direction():
    dir = [1 if IR_sensors[i].getValue() > 800 else 0 for i in range(3)]
    return dir
maze_array = np.zeros((20, 20))

# Define a 20x20 grid with all zeros (0 = unvisited, 1 = visited)
maze_map = [[0] * 20 for _ in range(20)]

# Initial robot position
robot_x, robot_y = 19, 10  # Assuming the robot starts at (0, 0)

def backtrack(previous_direction):
    """Turn the robot back to the original direction after backtracking"""
    if previous_direction == "LEFT":
        turnRight()  # Face back to original direction
    elif previous_direction == "RIGHT":
        turnLeft()  # Face back to original direction
    elif previous_direction == "UP":
        turnReverse()  # 180° turn to go back
    elif previous_direction == "DOWN":
        turnReverse()  # 180° turn to go back

# Directions mapping (dx, dy)
DIRECTION_MAP = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
}

def update_position(direction):
    """Update (x, y) based on movement direction"""
    global robot_x, robot_y

    dx, dy = DIRECTION_MAP[direction]
    robot_x += dx
    robot_y += dy

def search_maze(previous_direction=None):
    """Recursive maze search with proper backtracking"""
    # Main loop
def search_maze(direction):
    dir = get_direction()

    if direction == -1:
        turnLeft()
        moveForward()
    if direction == 0:
        moveForward()
    if direction == 1:
        turnRight()
        moveForward()
    

while robot.step(timestep) != -1:
        dir = get_direction()
        if dir[0] == 0:
            search_maze(-1)
        elif (dir[0]==1 and dir[1] == 0):
            search_maze(0)
        elif dir[0]==1 and dir[1] == 1 and dir[2] == 0:
            search_maze(1)
        else:
            turnReverse()


# Start the search

