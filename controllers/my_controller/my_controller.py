"""my_controller controller."""

from controller import Robot

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
us_names = ['us0', 'us1', 'us2']  # Front, Left, Right
ultrasonic_sensors = [get_device(name) for name in us_names]

# Function to convert ultrasonic readings (depends on the LUT)
def ultrasonic_to_distance(value):
    return value * 0.01  # Adjust scaling based on your lookup table

L_motor = robot.getDevice('left wheel motor')
R_motor = robot.getDevice('right wheel motor')

L_motor.setPosition(float('inf'))
R_motor.setPosition(float('inf'))

L_encoderVal = get_device('left wheel sensor')
R_encoderVal = get_device('right wheel sensor')
def encoderPID():
    #pid for straight moves
def moveForward():
    #move forward with pid for 0.25 meter
def turnRight():
    #turn 90 degree Right
def turnLeft():
    #turn 90 degree Left
def turnReverse():
    #turn 180 degree backward
def moveBack():
    #move 0.25 meter backward with pid

# Main loop
while robot.step(timestep) != -1:
    
    # Read ultrasonic sensors
    for us in ultrasonic_sensors:
        if us:
            raw_value = us.getValue()
            distance = ultrasonic_to_distance(raw_value)  # Convert to meters
            print(f"{us.getName()} raw: {raw_value}, distance: {distance:.3f}m")

    pass  # Keep simulation running
