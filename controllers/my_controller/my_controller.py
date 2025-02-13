"""my_controller controller."""

from controller import Robot
import math
import cv2
import numpy as np



# Create the Robot instance
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Directions mapping (dx, dy)
DIRECTION_MAP = [(-1,0),(0,1),(1,0),(0,-1)] # UP, RIGHT, DOWN, LEFT
maze_array = np.zeros((20, 20))

# Define a 20x20 grid with all zeros (0 = unvisited, 1 = visited)
maze_map = [[0] * 20 for _ in range(20)]

# Initial robot position
robot_x, robot_y = 19, 10  # Assuming the robot starts at (0, 0)


# Function to get device instances
def get_device(device_name):
    device = robot.getDevice(device_name)
    if device:
        device.enable(timestep)
    return device


# Get ultrasonic sensors
us_names = ['ps4', 'ps2', 'ps0','ps8']  # Front, Left, Right
IR_sensors = [get_device(name) for name in us_names]
side_sensor_names=['ps7','ps1','ps3','ps5']
IR_side_sensors = [get_device(name) for name in side_sensor_names]

L_encoder = get_device('left encoder')
R_encoder = get_device('right encoder')

L_motor = robot.getDevice('left motor')
R_motor = robot.getDevice('right motor')

L_motor.setPosition(float('inf'))
R_motor.setPosition(float('inf'))

inertial_unit =get_device('Inertial Unit')
gyro = get_device('gyroScope')

gps_device = get_device('GPS')
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

def adjest_positionF():
    
    IR_readings = [IR_sensors[i].getValue() for i in range(3)]
    if  800<IR_readings[1]:
        
        while robot.step(timestep) != -1:
            IR_readings = [IR_sensors[i].getValue() for i in range(3)]
            if IR_readings[1]-880!=0:
                deff = (IR_readings[1]-880)/abs(IR_readings[1]-880)
                print(deff,IR_readings[1])
                L_motor.setVelocity(-2*deff)
                R_motor.setVelocity(-2*deff)
                if 878<IR_readings[1]<882:
                    L_motor.setVelocity(0)
                    R_motor.setVelocity(0)
                    break
            else:
                break

def adjest_positionB():
    IR_readings = [IR_sensors[i].getValue() for i in range(4)]
    if  800<IR_readings[3]:
        
        while robot.step(timestep) != -1:
            IR_readings = [IR_sensors[i].getValue() for i in range(4)]
            if IR_readings[3]-880!=0:
                deff = (IR_readings[3]-880)/abs(IR_readings[3]-880)
                print(deff,IR_readings[3])
                L_motor.setVelocity(2*deff)
                R_motor.setVelocity(2*deff)
                if 878<IR_readings[3]<882:
                    L_motor.setVelocity(0)
                    R_motor.setVelocity(0)
                    break
            else:
                break

def adjest_positionRotation():
    
    side_IR_readings = [IR_side_sensors[i].getValue() for i in range(4)]
    print(side_IR_readings)
    if  800<side_IR_readings[0] and 800<side_IR_readings[1] :
            
            while robot.step(timestep) != -1:
                side_IR_readings = [IR_side_sensors[i].getValue() for i in range(4)]
                if (side_IR_readings[0]-side_IR_readings[1])!=0:
                    deff = (side_IR_readings[0]-side_IR_readings[1])/abs(side_IR_readings[0]-side_IR_readings[1])
                    print('adjesting rot left')
                    L_motor.setVelocity(-1*deff)
                    R_motor.setVelocity(1*deff)
                    if abs(side_IR_readings[0]-side_IR_readings[1])<2:
                        L_motor.setVelocity(0)
                        R_motor.setVelocity(0)
                        break
                else:
                    break
    elif  800<side_IR_readings[2] and 800<side_IR_readings[3] :
            
            while robot.step(timestep) != -1:
                side_IR_readings = [IR_side_sensors[i].getValue() for i in range(4)]
                if (side_IR_readings[2]-side_IR_readings[3])!=0:
                    deff = (side_IR_readings[2]-side_IR_readings[3])/abs(side_IR_readings[2]-side_IR_readings[3])
                    print('adjesting rot right', abs(side_IR_readings[2]-side_IR_readings[3]))

                    L_motor.setVelocity(-1*deff)
                    R_motor.setVelocity(1*deff)
                    if abs(side_IR_readings[2]-side_IR_readings[3])<2:
                        L_motor.setVelocity(0)
                        R_motor.setVelocity(0)
                        break
                else:
                    break


def moveForward():
    global adj_pos
    robot.step(timestep)
    start_gpsVal=gps_device.getValues()

    while robot.step(timestep) != -1:
        # Get encoder values and subtract offset
        current_gpsVal = gps_device.getValues()
        # if adj_pos:
        #     adjest_positionRotation()
        L_motor.setVelocity(10)
        R_motor.setVelocity(10)

        # Compute traveled distance
        distance_x,distance_y = abs(current_gpsVal[0]-start_gpsVal[0])*100,abs(current_gpsVal[1]-start_gpsVal[1])*100
        
        if(25<distance_x<26 or 25<distance_y<26):
           
            print("stop forward")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            robot.step(timestep*10)
            adjest_positionF()
            break

def moveBackward():
       
    robot.step(timestep)
    start_gpsVal=gps_device.getValues()

    while robot.step(timestep) != -1:
        # Get encoder values and subtract offset
        current_gpsVal = gps_device.getValues()
        L_motor.setVelocity(-10)
        R_motor.setVelocity(-10)

        # Compute traveled distance
        distance_x,distance_y = abs(current_gpsVal[0]-start_gpsVal[0])*100,abs(current_gpsVal[1]-start_gpsVal[1])*100
        
        if(25<distance_x<26 or 25<distance_y<26):
            stoping =True
            print("stop backward")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            robot.step(timestep*10)
            adjest_positionB()
            break

def turnRight():
    #turn 90 degree Right
    robot.step(timestep)
    initial_Ryaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])  # Initial yaw
    target_angleR = normalize_angle(initial_Ryaw - math.radians(90))  # Target yaw (-90°)

    print(f"Starting turn | Initial Yaw: {math.degrees(initial_Ryaw):.2f}° | Target Yaw: {math.degrees(target_angleR):.2f}°")

    L_motor.setVelocity(5)
    R_motor.setVelocity(-5)

    while robot.step(timestep) != -1:
        current_Ryaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])
        yaw_diffR = math.degrees(abs(angular_difference(target_angleR, current_Ryaw)))  # Fix wraparound

        #print(f"Yaw: {math.degrees(current_Ryaw):.2f}° | ΔYaw: {yaw_diffR:.2f}° ")

        if abs(yaw_diffR)<1:  # Stop when true yaw difference reaches 90°
            print("✅ Turn Complete! Stopping motors.")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            adjest_positionRotation()
            break

    robot.step(timestep * 10)
    
def turnLeft():
    #turn 90 degree Left
    robot.step(timestep)    
    print("turning left")
    initial_Lyaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])  # Extract yaw angle
   
    target_angleL= normalize_angle(initial_Lyaw+math.radians(90))  # Target yaw (-90 degrees)
    
    #print(f"Starting turn | Initial Yaw: {math.degrees(initial_Lyaw):.2f}° | Target Yaw: {math.degrees(target_angleL):.2f}°")
    
    L_motor.setVelocity(-5)
    R_motor.setVelocity(5)

    while(robot.step(timestep) != -1):
        cuurent_Lyaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])

        yaw_diffL = math.degrees(abs(angular_difference(target_angleL, cuurent_Lyaw))) 

        #print(f"Yaw: {math.degrees(cuurent_Lyaw):.2f}° | ΔYaw: {yaw_diffL:.2f}° ")
        
      
        if(abs(yaw_diffL)<1):
            print("stoping")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            adjest_positionRotation()
            break
    robot.step(timestep*10)



    robot.step(timestep * 10)  # Small delay after turning
       
def turnReverse():
    turnRight()
    turnRight()

def robotStop(getTime):
    L_motor.setVelocity(0)
    R_motor.setVelocity(0)
    robot.step(timestep*getTime)
    


def read_sensors():
    dir = [1 if IR_sensors[i].getValue() > 750 else 0 for i in range(3)]
    return dir




def backtrack(previous_direction):
    """Turn the robot back to the original direction and move forward"""
    moveBackward()
    if previous_direction == "LEFT":
        print("Backtracking LEFT")
        turnRight()

    elif previous_direction == "RIGHT":
        print("Backtracking RIGHT")
        turnLeft()
    else:
        print("Backtracking UP")

    
def update_position(direction,robot_x,robot_y):
    """Update (x, y) based on movement direction"""

    dx, dy = DIRECTION_MAP[direction]
    cell_x = robot_x + dx
    cell_y = robot_y + dy

    if maze_map[cell_x][cell_y] == 0:
        robot_x = cell_x
        robot_y = cell_y
        return (robot_x,robot_y)
    return None



def search_maze(robot_x,robot_y,setDirection,previous_direction=None):
    """Recursive maze search with proper backtracking"""
    global maze_map
    temp_Direction = setDirection
    if not (0 <= robot_x < 20 and 0 <= robot_y < 20):
        print("Out of bounds")
        return  # Out of bounds

    # Mark as visited
    maze_map[robot_x][robot_y] = 1
    for line in maze_map:
        print(line)
    print("Visited:", robot_x, robot_y)

    dir = read_sensors()  # Get sensor readings

    


    if dir[0] == 0:  # Left open
        print("Left open")
        new_direction = (temp_Direction - 1) % 4
        temp = update_position(new_direction,robot_x,robot_y)
        if (temp):
            turnLeft()
            setDirection=new_direction
            moveForward()
            search_maze(temp[0],temp[1],new_direction,"LEFT")

    if dir[1] == 0:  # Forward open
        print("Forward open")
        new_direction = temp_Direction
        temp = update_position(new_direction,robot_x,robot_y)
        if (temp):
            moveForward()
            search_maze(temp[0],temp[1],new_direction,"UP")

    if dir[2] == 0:  # Right open
        print("Right open")
        new_direction = (temp_Direction + 1) % 4
        temp = update_position(new_direction,robot_x,robot_y)
        if (temp):
            turnRight()
            setDirection=new_direction
            moveForward()
            search_maze(temp[0],temp[1],new_direction,"RIGHT")

    backtrack(previous_direction)  # Backtrack if no movement possible
adj_pos=False
moveForward()
adj_pos = True
search_maze(robot_x=19,robot_y=10,setDirection=0)
print("Maze Search completed")

robotStop(20)

