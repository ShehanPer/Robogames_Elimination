from controller import Robot
import math
import cv2
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())

print(timestep)
######## Main Variable ################

VISITING_MAP = [[0] * 20 for _ in range(20)] #20x20 grid (0 = unvisited, 1 = visited)

ENTRANCE=[19,10] # Entrance of the maze
DIRECTION_MAP = [(-1,0),(0,1),(1,0),(0,-1)] # UP, RIGHT, DOWN, LEFT
GREEN_CORDINATES = []

WALL_MAP = [[0] * 20 for _ in range(20)]
FLOOD_MAP1 = [[-1] * 20 for _ in range(20)]     #Flood fill map from entrance to first survivor
FLOOD_MAP2 = [[-1] * 20 for _ in range(20)]     #Flood fill map from first survivor to second survivor
FLOOD_MAP3 = [[-1] * 20 for _ in range(20)]     #Flood fill map from second survivor to third survivor
FLOOD_MAP4 = [[-1] * 20 for _ in range(20)]     #Flood fill map from third survivor to entrance

mark_pos=True   #Variable used when initialising the flood fill map.

num_boxes= 0    #calculate total cells travelled during dry run

########################################


def get_device(device_name):
    device = robot.getDevice(device_name)
    if device:
        device.enable(timestep)
    return device

################ Device Initializations #####################

# Get ultrasonic sensors
us_names = ['ps4', 'ps2', 'ps0','ps8']  
side_sensor_names=['ps7','ps1','ps3','ps5']

L_motor = robot.getDevice('left motor')
R_motor = robot.getDevice('right motor')

L_motor.setPosition(float('inf'))
R_motor.setPosition(float('inf'))

IR_sensors = [get_device(name) for name in us_names]
IR_side_sensors = [get_device(name) for name in side_sensor_names]

inertial_unit =get_device('Inertial Unit')
gyro = get_device('gyroScope')

gps_device = get_device('GPS')
camera=get_device('camera')

def normalize_angle(angle):
    """Normalize angle to the range [-π, π]"""
    return math.atan2(math.sin(angle), math.cos(angle))

def angular_difference(target, current):
    """Compute shortest difference between two angles in radians."""
    diff = target - current
    return (diff + math.pi) % (2 * math.pi) - math.pi  # Keep in range [-π, π]
####################################################################

def use_camera(cam):
    image =cam.getImage()
    width = cam.getWidth()
    height = cam.getHeight()

    image_array = np.frombuffer(image,dtype=np.uint8).reshape((height,width,4))
    img_rgb = image_array[:,:,:3]

    left_frame = img_rgb[:,:width//3]
    middle_frame = img_rgb[32*height//36:, width//3 : 2*width//3]
    right_frame = img_rgb[:,2*width//3:]

    # cv2.imshow('camera',img_rgb)
    # cv2.waitKey(1)
    return left_frame,middle_frame,right_frame




def check_grean(left_frame,right_frame):
    grean_pixel =(left_frame[:,:,1]>200) & (left_frame[:,:,0]<100) & (left_frame[:,:,2]<100)
    grean_pixel2 =(right_frame[:,:,1]>200) & (right_frame[:,:,0]<100) & (right_frame[:,:,2]<100)
    if (grean_pixel.any() or grean_pixel2.any()):
        if(IR_sensors[1].getValue()>800):
            print('grean found 🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢🟢')
            return True
    return False

def check_orange(middle_frame):
    orange_pixel =(middle_frame[:,:,0]<50) & (middle_frame[:,:,2]>200) # Detect orange and Red
    orange_ratio = np.sum(orange_pixel)/orange_pixel.size
    print(orange_ratio)
    if orange_ratio > 0.6:
        print('orange found 🟠🟠🟠🟠🟠🟠🟠🟠🟠🟠🟠🟠🟠🟠🟠🟠🟠🟠🟠🟠🟠🟠')        
        return True
    return False


def read_sensors():
    dir = [1 if IR_sensors[i].getValue() > 750 else 0 for i in range(4)]
    return dir


def update_position(direction,robot_x,robot_y):
    """Update (x, y) based on movement direction"""

    dx, dy = DIRECTION_MAP[direction]
    cell_x = robot_x + dx
    cell_y = robot_y + dy

    if VISITING_MAP[cell_x][cell_y] == 0:
        robot_x = cell_x
        robot_y = cell_y
        return (robot_x,robot_y)
    return None



def directionMap(previous_direction):
    store_direction = []
    Directions = read_sensors()
    if(previous_direction == 0):
        store_direction = [Directions[3],Directions[2],Directions[1],Directions[0]]
        return store_direction
    if(previous_direction == 1):
        store_direction = [Directions[2],Directions[1],Directions[0],Directions[3]]
        return store_direction
    if(previous_direction == 3):
        store_direction = [Directions[0],Directions[3],Directions[2],Directions[1]]
        return store_direction
    if(previous_direction == 2):
        store_direction = [Directions[1],Directions[0],Directions[3],Directions[2]]
        return store_direction
    
