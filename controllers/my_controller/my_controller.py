"""my_controller controller."""

from baseFunctions import *
from movements import *
import math


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
    global VISITING_MAP, WALL_MAP,GREEN_CORDINATES

    temp_Direction = setDirection
    lframe,mframe,rframe=use_camera(camera)

    if check_grean(lframe,rframe):
        GREEN_CORDINATES.append([robot_x,robot_y])

    if check_orange(mframe):
        store_Direction=[1,1,1,1]
    else:
        store_Direction = directionMap(setDirection)
    
    WALL_MAP[robot_x][robot_y] = store_Direction

    if not (0 <= robot_x < 20 and 0 <= robot_y < 20):
        print("Out of bounds")
        return  

    VISITING_MAP[robot_x][robot_y] = 1   # Mark as visited
   
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
        wallConfig[0]=WALL_MAP[robot_x][robot_y][0] | WALL_MAP[robot_x+1][robot_y][2]
    if robot_x!=0:
        wallConfig[2]=WALL_MAP[robot_x][robot_y][2] | WALL_MAP[robot_x-1][robot_y][0]
    if robot_y!=19:
        wallConfig[1]=WALL_MAP[robot_x][robot_y][1] | WALL_MAP[robot_x][robot_y+1][3]
    if robot_y!=0:
        wallConfig[3]=WALL_MAP[robot_x][robot_y][3] | WALL_MAP[robot_x][robot_y-1][1]

    return wallConfig
        
def update_wallMap():
    global WALL_MAP
    for i in range(20):
        for j in range(20):
            walls=getWalls(i,j)
            WALL_MAP[i][j]=walls

def update_Floodposition(direction,robot_x,robot_y,FLOOD_MAP):
    """Update (x, y) based on movement direction"""
    dx, dy = DIRECTION_MAP[direction]
    cell_x = robot_x + dx
    cell_y = robot_y + dy
   
    
    if FLOOD_MAP[cell_x][cell_y] == -1 or FLOOD_MAP[cell_x][cell_y] > FLOOD_MAP[robot_x][robot_y]:
        robot_x = cell_x
        robot_y = cell_y
        return (robot_x,robot_y)
    return None

def flood_maze(target,robot_pos,FLOOD_MAP):
    '''create flood array to find the shortest target to robot'''
    global WALL_MAP,mark_pos

    target_x,target_y = target

    if not (0 <= target_x < 20 and 0 <= target_y < 20):
        return  # Out of bounds
    if FLOOD_MAP[target_x][target_y]==robot_pos:
        return
    if mark_pos:
        FLOOD_MAP[target_x][target_y] = 0
        mark_pos=False
    cellWalls = WALL_MAP[target_x][target_y]  # Get sensor readings

    
    if cellWalls[0] == 0:  # down open
        temp = update_Floodposition(2,target_x,target_y,FLOOD_MAP)
        if (temp):
            FLOOD_MAP[temp[0]][temp[1]] = FLOOD_MAP[target_x][target_y]+1
            flood_maze((temp[0],temp[1]),robot_pos,FLOOD_MAP)

    if cellWalls[1] == 0:  # Right open
        temp = update_Floodposition(1,target_x,target_y,FLOOD_MAP)
        if (temp):
            FLOOD_MAP[temp[0]][temp[1]] = FLOOD_MAP[target_x][target_y]+1
            flood_maze((temp[0],temp[1]),robot_pos,FLOOD_MAP)

    if cellWalls[3] == 0:  # Left open
        temp = update_Floodposition(3,target_x,target_y,FLOOD_MAP)
        if (temp):
            FLOOD_MAP[temp[0]][temp[1]] = FLOOD_MAP[target_x][target_y]+1
            flood_maze((temp[0],temp[1]),robot_pos,FLOOD_MAP)
    
    if cellWalls[2] == 0:  # Up open
        temp = update_Floodposition(0,target_x,target_y,FLOOD_MAP)
        if temp is not None:
            FLOOD_MAP[temp[0]][temp[1]] = FLOOD_MAP[target_x][target_y]+1
            flood_maze((temp[0],temp[1]),robot_pos,FLOOD_MAP)



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


def floodfill_follow(start_pos,target_pos,direction,FLOOD_MAP):
    '''follow 0 position in flood array from current position'''
    global mark_pos
    print(direction)
    flood_maze(target_pos,start_pos,FLOOD_MAP)
    mark_pos=True
    for line in FLOOD_MAP:
        print(" ".join(f"{num:3}" for num in line))
    robot_x,robot_y = start_pos

    ## This while loop will take robot from start position to traget position
    facing_direction=direction #initial direction
    while FLOOD_MAP[robot_x][robot_y]!=0 and robot.step(timestep)!=-1 : 
        walls=WALL_MAP[robot_x][robot_y] 
        print("Now at ",(robot_x,robot_y))
        if robot_x<19 and FLOOD_MAP[robot_x+1][robot_y]==FLOOD_MAP[robot_x][robot_y]-1 and walls[0]!=1: #down
            floodfillturn(facing_direction,"DOWN")
            facing_direction="DOWN"
            robot_x+=1
            
        elif robot_x>0 and FLOOD_MAP[robot_x-1][robot_y]==FLOOD_MAP[robot_x][robot_y]-1 and walls[2]!=1: #up
            floodfillturn(facing_direction,"UP")
            facing_direction="UP"
            robot_x-=1
            
        elif robot_y<19 and FLOOD_MAP[robot_x][robot_y+1]==FLOOD_MAP[robot_x][robot_y]-1 and walls[1]!=1: #right
            floodfillturn(facing_direction,"RIGHT")
            facing_direction="RIGHT"
            robot_y+=1
            
        elif robot_y>0 and FLOOD_MAP[robot_x][robot_y-1]==FLOOD_MAP[robot_x][robot_y]-1 and walls[3]!=1: #left
            floodfillturn(facing_direction,"LEFT")
            facing_direction="LEFT"
            robot_y-=1

        else:
            print("Error : No path found")
            return
        print("Going to",(robot_x,robot_y))
        moveForward()
    return facing_direction

def find_next_nearest(coords, current):
    """Finds the nearest coordinate to the given current coordinate."""
    min_distance = float('inf')
    nearest_coord = None

    for coord in coords:
        distance = math.sqrt((current[0] - coord[0]) ** 2 + (current[1] - coord[1]) ** 2)
        if distance < min_distance:
            min_distance = distance
            nearest_coord = coord

    return nearest_coord, min_distance

def rearangeGreenCoordinates(GREEN_CORDINATES, ENTRANCE):
    """Rearranges coordinates using a nearest-neighbor approach."""
    if not GREEN_CORDINATES:
        return []

    current = ENTRANCE
    sorted_coordinates = []
    remaining = GREEN_CORDINATES.copy()  # Copy to avoid modifying the original list

    while remaining:
        next_coord, _ = find_next_nearest(remaining, current)
        sorted_coordinates.append(next_coord)
        remaining.remove(next_coord)
        current = next_coord  # Move to the next coordinate

    return sorted_coordinates  


moveForward() # Enter the maze
search_maze(ENTRANCE[0],ENTRANCE[1],0)  #Search whole maze and exit
update_wallMap()


for line in WALL_MAP:
    print(line)

GREEN_CORDINATES = rearangeGreenCoordinates(GREEN_CORDINATES, ENTRANCE)
print("Rearranged Coordinates:", GREEN_CORDINATES)

robotStop(500)

moveForward() # Get back into the maze


direction_1=floodfill_follow(ENTRANCE,GREEN_CORDINATES[0],"UP",FLOOD_MAP1)
robotStop(300) #Taking the survivor
direction_2=floodfill_follow(GREEN_CORDINATES[0],GREEN_CORDINATES[1],direction_1,FLOOD_MAP2)
robotStop(300) #Taking the survivor
direction_3=floodfill_follow(GREEN_CORDINATES[1],GREEN_CORDINATES[2],direction_2,FLOOD_MAP3)
robotStop(300) #Taking the survivor
direction_x=floodfill_follow(GREEN_CORDINATES[2],ENTRANCE,direction_3,FLOOD_MAP4)

#Exit the maze
turnLeft()
moveBackward()







