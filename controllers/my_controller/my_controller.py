"""my_controller controller."""

from baseFunctions import *




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


def adjest_positionF():
    
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

def adjest_positionB():
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

def adjest_positionRotation():
    
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
    global adj_pos,num_boxes
    robot.step(timestep)
    start_gpsVal=gps_device.getValues()
    num_boxes +=1
    while robot.step(timestep) != -1:
        # Get encoder values and subtract offset
        current_gpsVal = gps_device.getValues()
        # if adj_pos:
        #     adjest_positionRotation()
        L_motor.setVelocity(8)
        R_motor.setVelocity(8)
            
        
        # Compute traveled distance
        distance_x,distance_y = abs(current_gpsVal[0]-start_gpsVal[0])*100,abs(current_gpsVal[1]-start_gpsVal[1])*100
        
        if(25<distance_x<26 or 25<distance_y<26):
           
            #print("stop forward")
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            robot.step(timestep*10)
            adjest_positionF()
            break

def moveBackward():
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
            adjest_positionB()
            break

def turnRight():
    #turn 90 degree Right
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
            adjest_positionRotation()
            break

    robot.step(timestep * 10)
    
def turnLeft():
    #turn 90 degree Left
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
    

def backtrack(previous_direction):
    """Turn the robot back to the original direction and move forward"""
    moveBackward()
    if previous_direction == "LEFT":
        #print("Backtracking LEFT")
        turnRight()

    elif previous_direction == "RIGHT":
        #print("Backtracking RIGHT")
        turnLeft()
    else:
        print("")

    

def search_maze(robot_x,robot_y,setDirection,previous_direction=None):
    """Recursive maze search with proper backtracking"""
    global maze_map, direction_map

    temp_Direction = setDirection
    lframe,mframe,rframe=use_camera(camera)

    if check_grean(lframe,rframe):
        green_cordinates.append([robot_x,robot_y])

    if check_orange(mframe):
        store_Direction=[1,1,1,1]
    else:
        store_Direction = directionMap(setDirection)
    
    direction_map[robot_x][robot_y] = store_Direction

    if not (0 <= robot_x < 20 and 0 <= robot_y < 20):
        print("Out of bounds")
        return  # Out of bounds

    # Mark as visited
    maze_map[robot_x][robot_y] = 1
    # for line in maze_map:
    #     print(line)
    print("Visited:", robot_x, robot_y)

    dir = read_sensors()  # Get sensor readings


    if dir[0] == 0:  # Left open
        #print("Left open")
        new_direction = (temp_Direction - 1) % 4
        temp = update_position(new_direction,robot_x,robot_y)
        if (temp):
            turnLeft()
            setDirection=new_direction
            moveForward()
            search_maze(temp[0],temp[1],new_direction,"LEFT")

    if dir[1] == 0:  # Forward open
        #print("Forward open")
        new_direction = temp_Direction
        temp = update_position(new_direction,robot_x,robot_y)
        if (temp):
            moveForward()
            search_maze(temp[0],temp[1],new_direction,"UP")

    if dir[2] == 0:  # Right open
        #print("Right open")
        new_direction = (temp_Direction + 1) % 4
        temp = update_position(new_direction,robot_x,robot_y)
        if (temp):
            turnRight()
            setDirection=new_direction
            moveForward()
            search_maze(temp[0],temp[1],new_direction,"RIGHT")

    backtrack(previous_direction)  # Backtrack if no movement possible


def update_Floodposition(flood_array,direction,robot_x,robot_y):
    """Update (x, y) based on movement direction"""
    
    dx, dy = DIRECTION_MAP[direction]
    cell_x = robot_x + dx
    cell_y = robot_y + dy
   
    
    if flood_array[cell_x][cell_y] == 'x':
        robot_x = cell_x
        robot_y = cell_y
        return (robot_x,robot_y)
    return None

def flood_maze(robot_x,robot_y,flood_arr,setDirection,end_des,previous_direction=None):
    
    global maze_map, direction_map,mark_pos
    flood_array = flood_arr
    #temp_Direction = setDirection
    end_des = end_des
    if not (0 <= robot_x < 20 and 0 <= robot_y < 20):
        print("Out of bounds")
        return  # Out of bounds
    if flood_array[robot_x][robot_y]==end_des:
        return
    if mark_pos:
        flood_array[robot_x][robot_y] = 0
        mark_pos=False
    dir = direction_map[robot_x][robot_y]  # Get sensor readings

    
    if dir[0] == 0:  # down open
        print("down open")
        new_direction = 2
        temp = update_Floodposition(flood_array,new_direction,robot_x,robot_y)
        if (temp):
            flood_array[temp[0]][temp[1]] = flood_array[robot_x][robot_y]+1
            # setDirection=new_direction
            flood_maze(temp[0],temp[1],flood_array,new_direction,end_des,"DOWN")

    if dir[1] == 0:  # Forward open
        print("Right open")
        new_direction = 1
        temp = update_Floodposition(flood_array,new_direction,robot_x,robot_y)
        if (temp):
            flood_array[temp[0]][temp[1]] = flood_array[robot_x][robot_y]+1
            flood_maze(temp[0],temp[1],flood_array,new_direction,end_des,"RIGHT")
    if dir[3] == 0:  # Left open
        print("Left open")
        new_direction = 3
        temp = update_Floodposition(flood_array,new_direction,robot_x,robot_y)
        if (temp):
            flood_array[temp[0]][temp[1]] = flood_array[robot_x][robot_y]+1
            # setDirection=new_direction
            flood_maze(temp[0],temp[1],flood_array,new_direction,end_des,"LEFT")
    if dir[2] == 0:  # Up open
        print("up  open")
        new_direction = 0
       
        temp = update_Floodposition(flood_array,new_direction,robot_x,robot_y)
        
        if temp is not None:
            flood_array[temp[0]][temp[1]] = flood_array[robot_x][robot_y]+1
            # setDirection=new_direction
            flood_maze(temp[0],temp[1],flood_array,new_direction,end_des,"LEFT")

def go_robo():
    global flood_array01
    
    

moveForward()
search_maze(robot_x=19,robot_y=10,setDirection=0)
print("Maze Search completed")
print("Green Cordinates",green_cordinates)
print("Direction Map")
for line in direction_map:
    print(line)

print(num_boxes)

robotStop(20)
green_cordinates =[ [8, 0], [2, 18], [13, 19]]
direction_map =[[[0, 0, 1, 1], [1, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [0, 0, 1, 0], [0, 0, 1, 0], [0, 0, 1, 0], [0, 1, 1, 0], [0, 0, 1, 1], [0, 0, 1, 0], [0, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 0], [0, 0, 1, 0], [1, 0, 1, 0], [0, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0]],
[[0, 0, 0, 1], [0, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 1, 1], [1, 1, 0, 0], [1, 0, 0, 1], [1, 1, 0, 0], [1, 0, 0, 1], [1, 0, 0, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 0, 1, 0], [0, 1, 1, 0], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 0], [0, 0, 1, 1], [1, 1, 0, 0]],
[[0, 1, 0, 1], [0, 0, 0, 1], [1, 0, 1, 0], [0, 1, 1, 0], [1, 1, 0, 1], [1, 0, 0, 1], [0, 0, 1, 0], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 1, 0], [0, 1, 1, 0], [1, 0, 1, 1], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1], [1, 1, 0, 0], [0, 0, 0, 1], [1, 1, 1, 0], [1, 0, 0, 1], [1, 1, 1, 0]],
[[0, 1, 0, 1], [1, 0, 0, 1], [0, 1, 1, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 0, 0], [0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 1, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 0], [0, 0, 0, 1], [0, 0, 0, 0], [0, 1, 1, 0], [1, 0, 0, 1], [0, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0]],
[[0, 0, 0, 1], [1, 1, 1, 0], [1, 0, 0, 1], [1, 0, 1, 0], [0, 0, 1, 0], [0, 1, 1, 0], [1, 1, 0, 1], [1, 0, 0, 1], [1, 1, 0, 0], [0, 1, 0, 1], [0, 1, 0, 1], [1, 0, 0, 1], [0, 1, 1, 0], [0, 0, 0, 1], [1, 1, 0, 0], [0, 0, 0, 1], [0, 0, 1, 0], [1, 1, 0, 0], [1, 0, 1, 1], [1, 1, 0, 0]],
[[1, 0, 0, 1], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 0, 0], [1, 0, 0, 0], [1, 1, 1, 0], [0, 0, 1, 1], [1, 0, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 1, 1, 0], [1, 0, 0, 1], [1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1], [0, 1, 1, 0], [0, 0, 1, 1], [0, 1, 1, 0]],
[[0, 0, 1, 1], [0, 1, 1, 0], [0, 1, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 0, 0], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 0, 1], [0, 0, 1, 0], [0, 1, 1, 0], [1, 0, 0, 1], [0, 0, 0, 0], [0, 1, 0, 0], [1, 0, 0, 1], [1, 1, 0, 0], [0, 1, 0, 1]],
[[0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1], [1, 0, 0, 1], [1, 1, 0, 0], [1, 0, 1, 1], [0, 1, 1, 0], [1, 0, 1, 1], [1, 1, 0, 0], [0, 1, 0, 1], [1, 0, 0, 1], [0, 1, 0, 0], [0, 1, 1, 1], [0, 1, 0, 1], [1, 0, 0, 1], [1, 0, 1, 0], [0, 0, 1, 0], [1, 1, 0, 0]],
[[1, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1], [1, 0, 0, 1], [1, 1, 0, 0], [0, 0, 1, 1], [1, 0, 1, 0], [0, 1, 1, 0], [0, 1, 0, 1], [0, 0, 1, 1], [0, 1, 1, 0], [0, 0, 0, 1], [1, 0, 1, 0], [1, 0, 0, 0], [1, 0, 0, 0], [1, 1, 0, 0], [0, 0, 1, 1], [0, 1, 1, 0], [1, 0, 0, 1], [0, 1, 1, 0]],
[[0, 0, 1, 1], [1, 1, 0, 0], [0, 1, 0, 1], [0, 0, 1, 1], [1, 0, 1, 0], [1, 1, 0, 0], [0, 1, 1, 1], [1, 0, 0, 1], [0, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1], [1, 1, 0, 0], [0, 0, 1, 1], [1, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 1, 0, 0], [0, 1, 0, 1], [0, 1, 1, 1], [0, 1, 0, 1]],
[[0, 0, 0, 1], [0, 0, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 1, 1, 0], [1, 0, 1, 1], [0, 0, 0, 0], [1, 1, 1, 0], [0, 0, 0, 1], [0, 1, 0, 0], [0, 1, 0, 1], [0, 1, 1, 1], [1, 0, 0, 1], [1, 0, 1, 0], [0, 0, 1, 0], [0, 0, 1, 0], [0, 0, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 1, 0, 0]],
[[0, 0, 0, 1], [0, 1, 0, 0], [1, 0, 1, 1], [0, 1, 1, 0], [1, 0, 0, 1], [0, 1, 1, 0], [1, 0, 0, 1], [0, 0, 1, 0], [1, 1, 0, 0], [0, 1, 0, 1], [0, 1, 0, 1], [0, 0, 0, 1], [0, 1, 1, 0], [0, 1, 1, 1], [0, 0, 0, 1], [0, 1, 0, 0], [0, 0, 0, 1], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 0, 0]],
[[0, 1, 0, 1], [0, 0, 0, 1], [1, 0, 1, 0], [0, 1, 0, 0], [0, 0, 1, 1], [1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 1, 1], [1, 1, 0, 0], [1, 0, 0, 1], [1, 1, 0, 0], [1, 0, 0, 1], [0, 0, 0, 0], [0, 1, 0, 0], [0, 1, 0, 1], [0, 0, 0, 1], [0, 1, 1, 0], [1, 0, 1, 1], [0, 1, 0, 0]],
[[0, 1, 0, 1], [0, 0, 0, 1], [1, 1, 1, 0], [0, 1, 0, 1], [1, 1, 0, 1], [0, 0, 1, 1], [0, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1], [0, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 1, 1, 0], [1, 0, 0, 1], [0, 1, 0, 0], [1, 1, 0, 1], [1, 1, 0, 1], [0, 0, 0, 1], [0, 1, 1, 0], [1, 1, 0, 1]],
[[0, 1, 0, 1], [1, 0, 0, 1], [0, 1, 1, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 1, 0, 0], [0, 0, 0, 1], [1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 1, 0], [0, 0, 1, 0], [1, 1, 0, 0], [0, 0, 1, 1], [0, 1, 1, 0], [0, 0, 0, 1], [1, 0, 0, 0], [0, 1, 1, 0]],
[[1, 0, 0, 1], [0, 1, 1, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 0, 1, 0], [0, 0, 1, 0], [1, 1, 0, 0], [0, 0, 1, 1], [1, 0, 1, 0], [1, 0, 1, 0], [1, 1, 0, 0], [0, 0, 0, 1], [1, 1, 1, 0], [1, 0, 0, 1], [0, 1, 1, 0], [0, 1, 0, 1], [1, 0, 0, 1], [1, 0, 0, 0], [0, 1, 1, 0], [1, 1, 0, 1]],
[[0, 1, 1, 1], [1, 0, 0, 1], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 1, 0], [1, 1, 0, 0], [0, 1, 1, 1], [0, 0, 0, 1], [0, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 0, 1], [0, 1, 1, 0], [0, 1, 1, 1], [0, 0, 0, 1], [1, 0, 0, 0], [0, 1, 1, 0], [0, 1, 1, 1], [1, 0, 0, 1], [0, 1, 1, 0]],
[[1, 0, 0, 1], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1], [1, 0, 1, 0], [0, 1, 1, 0], [1, 0, 0, 1], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 1], [1, 1, 0, 0], [0, 1, 0, 1], [1, 0, 0, 1], [0, 1, 0, 0], [1, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 0], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 0]],
[[0, 1, 1, 1], [0, 1, 0, 1], [1, 1, 0, 1], [1, 0, 0, 1], [0, 1, 1, 0], [0, 1, 0, 1], [0, 1, 1, 1], [0, 0, 1, 1], [1, 1, 0, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 1, 0, 0], [0, 0, 1, 1], [1, 1, 0, 0], [0, 0, 1, 1], [0, 1, 0, 0], [0, 0, 1, 1], [1, 1, 0, 0], [0, 1, 0, 1], [0, 1, 1, 1]],
[[1, 0, 0, 1], [1, 0, 0, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [1, 1, 0, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 0, 1, 0], [1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 0, 0], [1, 0, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [1, 0, 0, 0], [1, 0, 1, 0], [1, 0, 0, 0], [1, 1, 0, 0]]]


# start_x,start_y = green_cordinates[1][0],green_cordinates[1][1]
# flood_maze(robot_x=start_x,robot_y=start_y,flood_arr=flood_array01,setDirection=0,end_des=[8,0])
# for line in flood_array01:
#     print(line)

