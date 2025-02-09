"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.



base_angular_V = 6.28 
wheelRadius = 0.03  # meters
trackWidth = 0.12  # Distance between the robot's wheels in meters
kp = 0.5  # Proportional gain
kd = 0.1  # Derivative gain
ki = 0.05  # Integral gain
tstep = int(robot.getBasicTimeStep()) # Time step for Webots controller (ms)

enL = 0
enR = 0
prox_sensors=[]
prox_readings=[0,0,0,0,0,0,0,0]


def initializeSensors():
    for ind in range(8):
        sensor='ps'+str(ind)
        prox_sensors.append(robot.getDevice(sensor))
        prox_sensors[ind].enable(tstep)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

def moveForward():
    left_motor.setVelocity(base_angular_V)
    right_motor.setVelocity(base_angular_V)

def stop():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)


def turnRight():
    left_motor.setVelocity(base_angular_V)
    right_motor.setVelocity(base_angular_V/8)

def turnLeft():

    left_motor.setVelocity(base_angular_V/8)
    right_motor.setVelocity(base_angular_V)

def rotateRight():
    left_motor.setVelocity(base_angular_V)
    right_motor.setVelocity(-base_angular_V)

def rotateLeft():
 
    left_motor.setVelocity(-base_angular_V)
    right_motor.setVelocity(base_angular_V)
    
def readSensors():
    for _ in range(8):
        prox_readings[_]=prox_sensors[_].getValue()

def runRobot():
    while robot.step(tstep) != -1:
        moveForward()

if __name__ == "__main__":

    # Access motors and encoders
    left_motor = robot.getDevice('left motor')
    right_motor = robot.getDevice('right motor')   

    left_encoder = robot.getDevice('left encoder')
    right_encoder = robot.getDevice('right encoder')

    # Set motors to velocity control mode
    left_motor.setPosition(float('inf'))  
    right_motor.setPosition(float('inf'))  
    left_motor.setVelocity(0)  
    right_motor.setVelocity(0) 
    initializeSensors()
    runRobot()

# Enter here exit cleanup code.
