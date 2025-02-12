"""my_controller controller."""

from controller import Robot
import math

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
us_names = ['ps0', 'ps1', 'ps2']  # Front, Left, Right
IR_sensors = [get_device(name) for name in us_names]

L_encoder = get_device('left encoder')
R_encoder = get_device('right encoder')

L_motor = robot.getDevice('left motor')
R_motor = robot.getDevice('right motor')

L_motor.setPosition(float('inf'))
R_motor.setPosition(float('inf'))

inertial_unit =get_device('inertial_unit')

# Function to convert ultrasonic readings (depends on the LUT)
def ir_to_distance(value):
    return value * 0.01  # Adjust scaling based on your lookup table

def normalize_angle(angle):
    """Normalize angle to the range [-π, π]"""
    return math.atan2(math.sin(angle), math.cos(angle))

def moveForward():
    # PID for straight moves
    global stoping

    stoping = False
    robot.step(timestep)

    # Get initial encoder values as offsets
    L_encoder_offset = L_encoder.getValue()
    R_encoder_offset = R_encoder.getValue()

    kp = 0.2
    ki = 0.0001
    kd = 0.01
    integral = 0
    derivative = 0
    last_error = 0

    LstartVal = 0  # Start at 0 since we're subtracting the offsets

    while robot.step(timestep) != -1:
        # Get encoder values and subtract offset
        L_encoderVal = L_encoder.getValue() - L_encoder_offset
        R_encoderVal = R_encoder.getValue() - R_encoder_offset

        error = (L_encoderVal - R_encoderVal) * 4
        integral += error
        derivative = error - last_error
        last_error = error
        correction = kp * error + ki * integral + kd * derivative
        correction = round(correction, 3)
        
        print("Correction:", correction)

        L_motor.setVelocity(2 - correction)
        R_motor.setVelocity(2 + correction)

        # Compute traveled distance
        LendVal = L_encoderVal  # Since we already subtracted offset
        distance = (LendVal - LstartVal) * 0.06 * 3.14 / 7
        distance = round(distance, 3)
        
        print('Distance:', distance)

        if(0.212<=distance<0.22):
            stoping =True
            print("stoping")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            robot.step(timestep*10)
            break

def turnRight():
    #turn 90 degree Right

    initial_Ryaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])  # Extract yaw angle
    #initial_L_encoder = L_encoder.getValue()
    #initial_R_encoder = R_encoder.getValue()

    target_angleR = initial_Ryaw - math.radians(90)  # Target yaw (-90 degrees)
    
    print(f"Starting turn | Initial Yaw: {math.degrees(initial_Ryaw):.2f}° | Target Yaw: {math.degrees(target_angleR):.2f}°")
    
    # Start turning
    L_motor.setVelocity(2)
    R_motor.setVelocity(-2)

    while robot.step(timestep) != -1:
        current_Ryaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])
        #current_L_encoder = L_encoder.getValue()
        #current_R_encoder = R_encoder.getValue()

        yaw_diffR = math.degrees(abs(current_Ryaw - initial_Ryaw))  # Difference in degrees
        #encoder_diff = abs(current_L_encoder - initial_L_encoder) + abs(current_R_encoder - initial_R_encoder)

        print(f"Yaw: {math.degrees(current_Ryaw):.2f}° | ΔYaw: {yaw_diffR:.2f}° ")

        # Stop when either yaw difference is ~90° OR encoder difference reaches threshold
        if yaw_diffR >= 90 :  
            print("✅ Turn Complete! Stopping motors.")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            break

    robot.step(timestep * 10) 
       

    
def turnLeft():
    #turn 90 degree Left
    initial_Lyaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])  # Extract yaw angle
    #L_encoderVal1,R_encoderVal1 = L_encoder.getValue(), R_encoder.getValue()

    target_angleL= initial_Lyaw+math.radians(90)  # Target yaw (-90 degrees)
    
    print(f"Starting turn | Initial Yaw: {math.degrees(initial_Lyaw):.2f}° | Target Yaw: {math.degrees(target_angleL):.2f}°")
    
    L_motor.setVelocity(-2)
    R_motor.setVelocity(2)

    while(robot.step(timestep) != -1):
        cuurent_Lyaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])

        yaw_diffL = math.degrees(abs(cuurent_Lyaw - initial_Lyaw))
        
        print(f"Yaw: {math.degrees(cuurent_Lyaw):.2f}° | ΔYaw: {yaw_diffL:.2f}° ")
        
        #L_encoderVal2,R_encoderVal2 = L_encoder.getValue(), R_encoder.getValue()
        #difR =abs( R_encoderVal2 - R_encoderVal1)
        #difL = abs(L_encoderVal2 - L_encoderVal1)
        
        if(yaw_diffL>=90):
            print("stoping")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            break
    robot.step(timestep*10)
       
def turnReverse():
    #turn 180 degree backward
    robot.step(timestep)
    L_encoderVal1,R_encoderVal1 = L_encoder.getValue(), R_encoder.getValue()
    while(robot.step(timestep) != -1):
        L_motor.setVelocity(2)
        R_motor.setVelocity(-2)
        L_encoderVal2,R_encoderVal2 = L_encoder.getValue(), R_encoder.getValue()
        difR =abs( R_encoderVal2 - R_encoderVal1)
        difL = abs(L_encoderVal2 - L_encoderVal1)
        print(difR,difL)
        if(difR>=6.9 or difL>=6.9):
            print("stoping")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            break
       
def moveBack():
    #move 0.25 meter backward with pid
    pass

# Main loop
while(robot.step(timestep) != -1):
    #moveForward()
    #turnLeft()
    #moveForward()
    #turnLeft()
    
    turnRight()
    


