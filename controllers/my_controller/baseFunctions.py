from controller import Robot
import math
import cv2
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())

maze_array = np.zeros((20, 20))

# Define a 20x20 grid with all zeros (0 = unvisited, 1 = visited)
maze_map = [[0] * 20 for _ in range(20)]

# Initial robot position
robot_x, robot_y = 19, 10  

DIRECTION_MAP = [(-1,0),(0,1),(1,0),(0,-1)] # UP, RIGHT, DOWN, LEFT

mark_pos=True
# Get ultrasonic sensors
us_names = ['ps4', 'ps2', 'ps0','ps8']  
side_sensor_names=['ps7','ps1','ps3','ps5']


######## Other global Variable ########
green_cordinates = []
direction_map = [[0] * 20 for _ in range(20)]
flood_array01 = [['x'] * 20 for _ in range(20)]

num_boxes= 0

########################################


def get_device(device_name):
    device = robot.getDevice(device_name)
    if device:
        device.enable(timestep)
    return device

################ Device Initializations #####################
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


####################################################################

def use_camera(cam):
    image =cam.getImage()
    width = cam.getWidth()
    height = cam.getHeight()

    image_array = np.frombuffer(image,dtype=np.uint8).reshape((height,width,4))
    img_rgb = image_array[:,:,:3]

    left_frame = img_rgb[:,:width//3]
    middle_frame = img_rgb[height//4:, width//3 : 2*width//3]
    right_frame = img_rgb[:,2*width//3:]

    cv2.imshow('camera',img_rgb)
    cv2.waitKey(1)
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
    orange_pixel =(middle_frame[:,:,0]>200) & (middle_frame[:,:,0]>150) & (middle_frame[:,:,2]<50)
    if orange_pixel.any():
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

    if maze_map[cell_x][cell_y] == 0:
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