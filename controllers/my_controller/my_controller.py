"""my_controller controller."""

from baseFunctions import *
from movements import *


def backtrack(previous_direction):
    """make the robot to follow the path it came from """
    moveBackward()
    if previous_direction == "LEFT":
        turnRight()

    elif previous_direction == "RIGHT":
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
        return  

    maze_map[robot_x][robot_y] = 1   # Mark as visited
   
    print("Visited:", robot_x, robot_y)

    dir = read_sensors()  # Get sensor readings


    if dir[0] == 0:  # Left open
        new_direction = (temp_Direction - 1) % 4
        temp = update_position(new_direction,robot_x,robot_y)
        if (temp):
            turnLeft()
            setDirection=new_direction
            moveForward()
            search_maze(temp[0],temp[1],new_direction,"LEFT")

    if dir[1] == 0:  # Forward open
        new_direction = temp_Direction
        temp = update_position(new_direction,robot_x,robot_y)
        if (temp):
            moveForward()
            search_maze(temp[0],temp[1],new_direction,"UP")

    if dir[2] == 0:  # Right open
        new_direction = (temp_Direction + 1) % 4
        temp = update_position(new_direction,robot_x,robot_y)
        if (temp):
            turnRight()
            setDirection=new_direction
            moveForward()
            search_maze(temp[0],temp[1],new_direction,"RIGHT")

    backtrack(previous_direction)  # Backtrack if no movement possible



def getWalls(robot_x,robot_y):
    '''return the wall configuration around the cell including effect of fire pits'''
    wallConfig=[1,1,1,1]
    if robot_x!=19:
        wallConfig[0]=direction_map[robot_x][robot_y][0] | direction_map[robot_x+1][robot_y][2]
    if robot_x!=0:
        wallConfig[2]=direction_map[robot_x][robot_y][2] | direction_map[robot_x-1][robot_y][0]
    if robot_y!=19:
        wallConfig[1]=direction_map[robot_x][robot_y][1] | direction_map[robot_x][robot_y+1][3]
    if robot_y!=0:
        wallConfig[3]=direction_map[robot_x][robot_y][3] | direction_map[robot_x][robot_y-1][1]

    return wallConfig
        

def update_Floodposition(direction,robot_x,robot_y):
    """Update (x, y) based on movement direction"""
    global flood_array
    dx, dy = DIRECTION_MAP[direction]
    cell_x = robot_x + dx
    cell_y = robot_y + dy
   
    
    if flood_array[cell_x][cell_y] == -1:
        robot_x = cell_x
        robot_y = cell_y
        return (robot_x,robot_y)
    return None

def flood_maze(target,end_des):
    '''create flood array to find the shortest path to target'''
    global maze_map, direction_map,mark_pos,flood_array

    target_x,target_y = target

    if not (0 <= target_x < 20 and 0 <= target_y < 20):
        return  # Out of bounds
    if flood_array[target_x][target_y]==end_des:
        return
    if mark_pos:
        flood_array[target_x][target_y] = 0
        mark_pos=False
    cellWalls = getWalls(target_x,target_y)  # Get sensor readings

    
    if cellWalls[0] == 0:  # down open
        temp = update_Floodposition(2,target_x,target_y)
        if (temp):
            flood_array[temp[0]][temp[1]] = flood_array[target_x][target_y]+1
            flood_maze((temp[0],temp[1]),end_des)

    if cellWalls[1] == 0:  # Right open
        temp = update_Floodposition(1,target_x,target_y)
        if (temp):
            flood_array[temp[0]][temp[1]] = flood_array[target_x][target_y]+1
            flood_maze((temp[0],temp[1]),end_des)

    if cellWalls[3] == 0:  # Left open
        temp = update_Floodposition(3,target_x,target_y)
        if (temp):
            flood_array[temp[0]][temp[1]] = flood_array[target_x][target_y]+1
            # setDirection=new_direction
            flood_maze((temp[0],temp[1]),end_des)
    
    if cellWalls[2] == 0:  # Up open
        temp = update_Floodposition(0,target_x,target_y)
        if temp is not None:
            flood_array[temp[0]][temp[1]] = flood_array[target_x][target_y]+1
            # setDirection=new_direction
            flood_maze((temp[0],temp[1]),end_des)



def floodfillturn(current_direction,next_direction):
    '''Turning logic when following floodfill path'''
    if current_direction=="UP":
        if next_direction=="DOWN":
            turnReverse()
        elif next_direction=="LEFT":
            turnLeft()
        elif next_direction=="RIGHT":
            turnRight()
    
    elif current_direction=="DOWN":
        if next_direction=="UP":
            turnReverse()
        elif next_direction=="LEFT":
            turnRight()
        elif next_direction=="RIGHT":
            turnLeft()
    
    elif current_direction=="LEFT":
        if next_direction=="RIGHT":
            turnReverse()
        elif next_direction=="UP":
            turnRight()
        elif next_direction=="DOWN":
            turnLeft()
    
    elif current_direction=="RIGHT":
        if next_direction=="LEFT":
            turnReverse()
        elif next_direction=="UP":
            turnLeft()
        elif next_direction=="DOWN":
            turnRight()


def floodfill_follow():
    '''follow 0 position in flood array from current position'''
    
    global flood_array
    flood_maze(green_cordinates[0],end_des=[19,10])
    for line in flood_array:
        print(line)
    robot_x,robot_y = (19,10)
    ## This while loop will take robot from initial position to first survivor
    facing_direction="UP" #initial direction
    while flood_array[robot_x][robot_y]!=0 and robot.step(timestep)!=-1 : 
        walls=getWalls(robot_x,robot_y) 
        if robot_x<19 and flood_array[robot_x+1][robot_y]==flood_array[robot_x][robot_y]-1 and walls[0]!=1: #down
            floodfillturn(facing_direction,"DOWN")
            facing_direction="DOWN"
            robot_x+=1
            
        elif robot_x>0 and flood_array[robot_x-1][robot_y]==flood_array[robot_x][robot_y]-1 and walls[2]!=1: #up
            floodfillturn(facing_direction,"UP")
            facing_direction="UP"
            robot_x-=1
            
        elif robot_y<19 and flood_array[robot_x][robot_y+1]==flood_array[robot_x][robot_y]-1 and walls[1]!=1: #right
            floodfillturn(facing_direction,"RIGHT")
            facing_direction="RIGHT"
            robot_y+=1
            
        elif robot_y>0 and flood_array[robot_x][robot_y-1]==flood_array[robot_x][robot_y]-1 and walls[3]!=1: #left
            floodfillturn(facing_direction,"LEFT")
            facing_direction="LEFT"
            robot_y-=1

        else:
            print("Error : No path found")
            return
        moveForward()


    robotStop(30)    
    flood_array = [[-1] * 20 for _ in range(20)] #reset flood array
    flood_maze(green_cordinates[1],green_cordinates[0])   #search for best path to next survivor
    for line in flood_array:
        print(line)
   ## This while loop will take robot from first survivor to next survivor
    while flood_array[robot_x][robot_y]!=0 and robot.step(timestep)!=-1 : 
        walls=getWalls(robot_x,robot_y) 
        if robot_x<19 and flood_array[robot_x+1][robot_y]==flood_array[robot_x][robot_y]-1 and walls[0]!=1: #down
            floodfillturn(facing_direction,"DOWN")
            facing_direction="DOWN"
            robot_x+=1
            
        elif robot_x>0 and flood_array[robot_x-1][robot_y]==flood_array[robot_x][robot_y]-1 and walls[2]!=1: #up
            floodfillturn(facing_direction,"UP")
            facing_direction="UP"
            robot_x-=1
            
        elif robot_y<19 and flood_array[robot_x][robot_y+1]==flood_array[robot_x][robot_y]-1 and walls[1]!=1: #right
            floodfillturn(facing_direction,"RIGHT")
            facing_direction="RIGHT"
            robot_y+=1
            
        elif robot_y>0 and flood_array[robot_x][robot_y-1]==flood_array[robot_x][robot_y]-1 and walls[3]!=1: #left
            floodfillturn(facing_direction,"LEFT")
            facing_direction="LEFT"
            robot_y-=1

        else:
            print("Error : No path found")
            return
        moveForward()


    robotStop(30)  
    flood_array = [[-1] * 20 for _ in range(20)] #reset flood array
    flood_maze(green_cordinates[2],green_cordinates[1])   #search for best path to last survivor
    for line in flood_array:
        print(line)    
   ## This while loop will take robot to last survivor
    while flood_array[robot_x][robot_y]!=0 and robot.step(timestep)!=-1 : 
        walls=getWalls(robot_x,robot_y) 
        if robot_x<19 and flood_array[robot_x+1][robot_y]==flood_array[robot_x][robot_y]-1 and walls[0]!=1: #down
            floodfillturn(facing_direction,"DOWN")
            facing_direction="DOWN"
            robot_x+=1
            
        elif robot_x>0 and flood_array[robot_x-1][robot_y]==flood_array[robot_x][robot_y]-1 and walls[2]!=1: #up
            floodfillturn(facing_direction,"UP")
            facing_direction="UP"
            robot_x-=1
            
        elif robot_y<19 and flood_array[robot_x][robot_y+1]==flood_array[robot_x][robot_y]-1 and walls[1]!=1: #right
            floodfillturn(facing_direction,"RIGHT")
            facing_direction="RIGHT"
            robot_y+=1
            
        elif robot_y>0 and flood_array[robot_x][robot_y-1]==flood_array[robot_x][robot_y]-1 and walls[3]!=1: #left
            floodfillturn(facing_direction,"LEFT")
            facing_direction="LEFT"
            robot_y-=1

        else:
            print("Error : No path found")
            return
        moveForward()

    robotStop(30)

    flood_array = [[-1] * 20 for _ in range(20)] #reset flood array
    flood_maze((19,10),green_cordinates[2])   #search for best path to get back at start position
    for line in flood_array:
        print(line)
    ## This while loop will take robot back to fisrt position
    while flood_array[robot_x][robot_y]!=0 and robot.step(timestep)!=-1 : 
        walls=getWalls(robot_x,robot_y) 
        if robot_x<19 and flood_array[robot_x+1][robot_y]==flood_array[robot_x][robot_y]-1 and walls[0]!=1: #down
            floodfillturn(facing_direction,"DOWN")
            facing_direction="DOWN"
            robot_x+=1
            
        elif robot_x>0 and flood_array[robot_x-1][robot_y]==flood_array[robot_x][robot_y]-1 and walls[2]!=1: #up
            floodfillturn(facing_direction,"UP")
            facing_direction="UP"
            robot_x-=1
            
        elif robot_y<19 and flood_array[robot_x][robot_y+1]==flood_array[robot_x][robot_y]-1 and walls[1]!=1: #right
            floodfillturn(facing_direction,"RIGHT")
            facing_direction="RIGHT"
            robot_y+=1
            
        elif robot_y>0 and flood_array[robot_x][robot_y-1]==flood_array[robot_x][robot_y]-1 and walls[3]!=1: #left
            floodfillturn(facing_direction,"LEFT")
            facing_direction="LEFT"
            robot_y-=1

        else:
            print("Error : No path found")
            return
        moveForward()




robotStop(20)
green_cordinates =[ [8, 0], [2, 18], [13, 19]]
direction_map =[[[0, 0, 1, 1], [1, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [0, 0, 1, 0], [0, 0, 1, 0], [0, 0, 1, 0], [0, 1, 1, 0], [0, 0, 1, 1], [0, 0, 1, 0], [0, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 0], [0, 0, 1, 0], [1, 0, 1, 0], [0, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0]],
[[0, 0, 0, 1], [0, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 1, 1], [1, 1, 0, 0], [1, 0, 0, 1], [1, 1, 0, 0], [1, 0, 0, 1], [1, 0, 0, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 0, 1, 0], [0, 1, 1, 0], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 0], [0, 0, 1, 1], [1, 1, 0, 0]],
[[0, 1, 0, 1], [0, 0, 0, 1], [1, 0, 1, 0], [0, 1, 1, 0], [1, 1, 0, 1], [1, 0, 0, 1], [0, 0, 1, 0], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 1, 0], [0, 1, 1, 0], [1, 0, 1, 1], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1], [1, 1, 0, 0], [0, 0, 0, 1], [1, 1, 1, 0], [1, 0, 0, 1], [1, 1, 1, 0]],
[[0, 1, 0, 1], [1, 0, 0, 1], [0, 1, 1, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 0, 0], [0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 1, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 0], [0, 0, 0, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [0, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0]],
[[0, 0, 0, 1], [1, 1, 1, 0], [1, 0, 0, 1], [1, 0, 1, 0], [0, 0, 1, 0], [0, 1, 1, 0], [1, 1, 0, 1], [1, 0, 0, 1], [1, 1, 0, 0], [0, 1, 0, 1], [0, 1, 0, 1], [1, 0, 0, 1], [0, 1, 1, 0], [0, 0, 0, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 0, 0], [1, 0, 1, 1], [1, 1, 0, 0]],
[[1, 0, 0, 1], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 0, 0], [1, 0, 0, 0], [1, 1, 1, 0], [0, 0, 1, 1], [1, 0, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 1, 1, 0], [1, 0, 0, 1], [1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [0, 1, 1, 0], [0, 0, 1, 1], [0, 1, 1, 0]],
[[0, 0, 1, 1], [0, 1, 1, 0], [0, 1, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 0, 0], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 0, 1], [0, 0, 1, 0], [0, 1, 1, 0], [1, 0, 0, 1], [0, 0, 0, 0], [0, 1, 0, 0], [1, 0, 0, 1], [1, 1, 0, 0], [0, 1, 0, 1]],
[[0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1], [1, 0, 0, 1], [1, 1, 0, 0], [1, 0, 1, 1], [0, 1, 1, 0], [1, 0, 1, 1], [1, 1, 0, 0], [0, 1, 0, 1], [1, 0, 0, 1], [0, 1, 0, 0], [0, 1, 1, 1], [0, 1, 0, 1], [1, 0, 0, 1], [1, 0, 1, 0], [0, 0, 1, 0], [1, 1, 0, 0]],
[[1, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1], [1, 0, 0, 1], [1, 1, 0, 0], [0, 0, 1, 1], [1, 0, 1, 0], [0, 1, 1, 0], [0, 1, 0, 1], [0, 0, 1, 1], [0, 1, 1, 0], [0, 0, 0, 1], [1, 0, 1, 0], [1, 0, 0, 0], [1, 0, 0, 0], [1, 1, 0, 0], [0, 0, 1, 1], [0, 1, 1, 0], [1, 0, 0, 1], [0, 1, 1, 0]],
[[0, 0, 1, 1], [1, 1, 0, 0], [0, 1, 0, 1], [0, 0, 1, 1], [1, 0, 1, 0], [1, 1, 0, 0], [0, 1, 1, 1], [1, 0, 0, 1], [0, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1], [1, 1, 0, 0], [0, 0, 1, 1], [1, 0, 1, 0], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [0, 1, 0, 1], [0, 1, 1, 1], [0, 1, 0, 1]],
[[0, 0, 0, 1], [0, 0, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 1, 1, 0], [1, 0, 1, 1], [0, 0, 0, 0], [1, 1, 1, 0], [0, 0, 0, 1], [0, 1, 0, 0], [0, 1, 0, 1], [0, 1, 1, 1], [1, 0, 0, 1], [1, 0, 1, 0], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 0, 0], [1, 0, 0, 1], [0, 1, 0, 0]],
[[0, 0, 0, 1], [0, 1, 0, 0], [1, 0, 1, 1], [0, 1, 1, 0], [1, 0, 0, 1], [0, 1, 1, 0], [1, 0, 0, 1], [0, 0, 1, 0], [1, 1, 0, 0], [0, 1, 0, 1], [0, 1, 0, 1], [0, 0, 0, 1], [0, 1, 1, 0], [0, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 0, 0]],
[[0, 1, 0, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [0, 0, 1, 1], [1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 1, 1], [1, 1, 0, 0], [1, 0, 0, 1], [1, 1, 0, 0], [1, 0, 0, 1], [0, 0, 0, 0], [0, 1, 0, 0], [0, 1, 0, 1], [0, 0, 0, 1], [0, 1, 1, 0], [1, 0, 1, 1], [0, 1, 0, 0]],
[[0, 1, 0, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 0, 1], [0, 0, 1, 1], [0, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1], [0, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 1, 1, 0], [1, 0, 0, 1], [0, 1, 0, 0], [1, 1, 0, 1], [1, 1, 0, 1], [0, 0, 0, 1], [0, 1, 1, 0], [1, 1, 0, 1]],
[[0, 1, 0, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 0, 1, 0], [1, 1, 0, 0], [0, 0, 0, 1], [1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 1, 0], [0, 0, 1, 0], [1, 1, 0, 0], [0, 0, 1, 1], [0, 1, 1, 0], [0, 0, 0, 1], [1, 0, 0, 0], [0, 1, 1, 0]],
[[1, 0, 0, 1], [0, 1, 1, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 0, 1, 0], [0, 0, 1, 0], [1, 1, 0, 0], [0, 0, 1, 1], [1, 0, 1, 0], [1, 0, 1, 0], [1, 1, 0, 0], [0, 0, 0, 1], [1, 1, 1, 0], [1, 0, 0, 1], [0, 1, 1, 0], [0, 1, 0, 1], [1, 0, 0, 1], [1, 0, 0, 0], [0, 1, 1, 0], [1, 1, 0, 1]],
[[0, 1, 1, 1], [1, 0, 0, 1], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 1, 0], [1, 1, 0, 0], [0, 1, 1, 1], [0, 0, 0, 1], [0, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 0, 1], [0, 1, 1, 0], [0, 1, 1, 1], [0, 0, 0, 1], [1, 0, 0, 0], [0, 1, 1, 0], [0, 1, 1, 1], [1, 0, 0, 1], [0, 1, 1, 0]],
[[1, 0, 0, 1], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1], [1, 0, 1, 0], [0, 1, 1, 0], [1, 0, 0, 1], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 1], [1, 1, 0, 0], [0, 1, 0, 1], [1, 0, 0, 1], [0, 1, 0, 0], [1, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 0], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 0]],
[[0, 1, 1, 1], [0, 1, 0, 1], [1, 1, 0, 1], [1, 0, 0, 1], [0, 1, 1, 0], [0, 1, 0, 1], [0, 1, 1, 1], [0, 0, 1, 1], [1, 1, 0, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 1, 0, 0], [0, 0, 1, 1], [1, 1, 0, 0], [0, 0, 1, 1], [0, 1, 0, 0], [0, 0, 1, 1], [1, 1, 0, 0], [0, 1, 0, 1], [0, 1, 1, 1]],
[[1, 0, 0, 1], [1, 0, 0, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [1, 1, 0, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0], [1, 0, 1, 1], [1, 0, 0, 0], [1, 0, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [1, 0, 0, 0], [1, 0, 1, 0], [1, 0, 0, 0], [1, 1, 0, 0]]]



floodfill_follow()


