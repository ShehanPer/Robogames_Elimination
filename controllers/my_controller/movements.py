from baseFunctions import *

def adjust_positionF():
    '''Adjust the position of the robot to avoid deviations in position when moving forward'''
    IR_readings = [IR_sensors[i].getValue() for i in range(3)]
    if  800<IR_readings[1]:
        
        while robot.step(timestep) != -1:
            IR_readings = [IR_sensors[i].getValue() for i in range(3)]
            if IR_readings[1]-880!=0:
                deff = (IR_readings[1]-880)/abs(IR_readings[1]-880)
                L_motor.setVelocity(-2*deff)
                R_motor.setVelocity(-2*deff)
                if 878<IR_readings[1]<882:
                    L_motor.setVelocity(0)
                    R_motor.setVelocity(0)
                    break
            else:
                break

def adjust_positionB():
    '''Adjust the position of the robot to avoid deviations in position when moving Backward'''
    IR_readings = [IR_sensors[i].getValue() for i in range(4)]
    if  800<IR_readings[3]:
        
        while robot.step(timestep) != -1:
            IR_readings = [IR_sensors[i].getValue() for i in range(4)]

            if IR_readings[3]-880!=0:
                deff = (IR_readings[3]-880)/abs(IR_readings[3]-880)
                L_motor.setVelocity(2*deff)
                R_motor.setVelocity(2*deff)

                if 878<IR_readings[3]<882:
                    L_motor.setVelocity(0)
                    R_motor.setVelocity(0)
                    break
            else:
                break

def adjust_positionRotation():
    '''Adjust the position of the robot to avoid deviations in angle'''
    side_IR_readings = [IR_side_sensors[i].getValue() for i in range(4)]
    #print(side_IR_readings)
    if  800<side_IR_readings[0] and 800<side_IR_readings[1] :
            
            while robot.step(timestep) != -1:
                side_IR_readings = [IR_side_sensors[i].getValue() for i in range(4)]
                if (side_IR_readings[0]-side_IR_readings[1])!=0:
                    deff = (side_IR_readings[0]-side_IR_readings[1])/abs(side_IR_readings[0]-side_IR_readings[1])
                    # print('adjesting rot left')
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
                    #print('adjesting rot right', abs(side_IR_readings[2]-side_IR_readings[3]))

                    L_motor.setVelocity(-1*deff)
                    R_motor.setVelocity(1*deff)
                    if abs(side_IR_readings[2]-side_IR_readings[3])<2:
                        L_motor.setVelocity(0)
                        R_motor.setVelocity(0)
                        break
                else:
                    break


def moveForward():
    '''move the robot forward to next cell'''
    global adj_pos,num_boxes
    robot.step(timestep)
    start_gpsVal=gps_device.getValues()
    num_boxes +=1
    while robot.step(timestep) != -1:
        # Get encoder values and subtract offset
        current_gpsVal = gps_device.getValues()

        L_motor.setVelocity(8)
        R_motor.setVelocity(8)
            
        
        # Compute traveled distance
        distance_x,distance_y = abs(current_gpsVal[0]-start_gpsVal[0])*100,abs(current_gpsVal[1]-start_gpsVal[1])*100
        
        if(25<distance_x<26 or 25<distance_y<26):
           
            #print("stop forward")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            robot.step(timestep*10)
            adjust_positionF()
            break

def moveBackward():
    '''Move the robot backward to previous cell'''
    global num_boxes 
    robot.step(timestep)
    start_gpsVal=gps_device.getValues()
    num_boxes +=1
    while robot.step(timestep) != -1:
        # Get encoder values and subtract offset
        current_gpsVal = gps_device.getValues()
        L_motor.setVelocity(-8)
        R_motor.setVelocity(-8)

        # Compute traveled distance
        distance_x,distance_y = abs(current_gpsVal[0]-start_gpsVal[0])*100,abs(current_gpsVal[1]-start_gpsVal[1])*100
        
        if(25<distance_x<26 or 25<distance_y<26):
            stoping =True
            #print("stop backward")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            robot.step(timestep*10)
            adjust_positionB()
            break

def turnRight():
    '''make a 90 degree right turn'''
    robot.step(timestep)
    initial_Ryaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])  # Initial yaw
    target_angleR = normalize_angle(initial_Ryaw - math.radians(90))  # Target yaw (-90°)

    L_motor.setVelocity(5)
    R_motor.setVelocity(-5)

    while robot.step(timestep) != -1:
        current_Ryaw = normalize_angle(inertial_unit.getRollPitchYaw()[2])
        yaw_diffR = math.degrees(abs(angular_difference(target_angleR, current_Ryaw)))  # Fix wraparound

        if abs(yaw_diffR)<1:  # Stop when true yaw difference reaches 90°
            #print("Turn right Complete! Stopping motors.")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            adjust_positionRotation()
            break

    robot.step(timestep * 10)
    
def turnLeft():
    '''make a 90 degree left turn'''
    robot.step(timestep)    
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
            #print("Turn left Complete! Stopping motors.")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            adjust_positionRotation()
            break
    robot.step(timestep*10)



    robot.step(timestep * 10)  # Small delay after turning
       
def turnReverse():
    '''make a 180 degree turn'''
    turnRight()
    turnRight()

def robotStop(getTime):
    '''Stop the robot for a given time'''
    L_motor.setVelocity(0)
    R_motor.setVelocity(0)
    robot.step(timestep*getTime)