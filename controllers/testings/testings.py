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
us_names = ['ps4', 'ps2', 'ps0']  # Front, Left, Right
IR_sensors = [get_device(name) for name in us_names]

L_encoder = get_device('left encoder')
R_encoder = get_device('right encoder')

L_motor = robot.getDevice('left motor')
R_motor = robot.getDevice('right motor')

L_motor.setPosition(float('inf'))
R_motor.setPosition(float('inf'))

L_motor.setVelocity(0.0)
R_motor.setVelocity(0.0)

inertial_unit =get_device('Inertial Unit')
gyro = get_device('gyroScope')

gps=get_device('GPS')

camera=get_device('camera')


def use_camera(cam):
    image =cam.getImage()
    width = cam.getWidth()
    height = cam.getHeight()

    image_array = np.frombuffer(image,dtype=np.uint8).reshape((height,width,4))
    img_rgb = image_array[:,:,:3]
    left_frame = img_rgb[:,:width//3]
    middle_frame = img_rgb[32*height//36:, width//3 : 2*width//3]
    right_frame = img_rgb[:,2*width//3:]


    cv2.imshow('camera',img_rgb)
    cv2.waitKey(1)
    robot.step(timestep)
    cv2.imshow('camera',left_frame)
    cv2.waitKey(1)
    robot.step(timestep)
    cv2.imshow('camera',middle_frame)
    cv2.waitKey(1)
    robot.step(timestep)
    cv2.imshow('camera',right_frame)
    cv2.waitKey(1)
    return left_frame, middle_frame, right_frame





while robot.step(timestep) != -1:
    
    lframe,mframe,rframe=use_camera(camera)
    print("Left Frame")
    print(lframe)
    print("Middle Frame")
    print(mframe)
    print("Right Frame")
    print(rframe)
    robot.step(timestep*5)
 