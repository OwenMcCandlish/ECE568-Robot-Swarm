##########################################################################################################
# Imports
from PIL import Image
import numpy as np
import math
import config

##########################################################################################################
# Globals
PATHS = [ [], [], [] ]          # Global Variable for accessing the planned paths for each bot


##########################################################################################################
"""
    Task 1: Read Image
    Lead: Zohaib
"""

def template1():
    print(f"Create functions here")

# End Task 1
##########################################################################################################



##########################################################################################################
"""
    Task 2: Create Map
    Lead: Zohaib
"""

def template2():
    print(f"Create functions here")

# End Task 2
##########################################################################################################


##########################################################################################################
"""
    Task 3: Locate Robots
    Lead: Zohaib
"""

def template3():
    print(f"Create functions here")

# End Task 3
##########################################################################################################




##########################################################################################################
"""
    Task 4: Plan Paths
    Lead: Joshua
    Requires:  
                start   <list>          x,y coordinates of the leader's current position
                end     <list>          x,y coordinates of the leader's ending point
                f1      <list>          x,y coordinates of follower one's position
                f2      <list>          x,y coordinates of follower two's position
    Returns:    paths   <list>          2D list of paths [ [path1], [path2], [path3] ]
"""

def create_one_path(start, end):
    min_num_points = np.linalg.norm(end - start).astype(int) + 1
    line = np.linspace(start, end, min_num_points)
    line_rounded = np.round(line).astype(int)
    line_unique = np.unique(line_rounded, axis=0)
    return line_unique.tolist()

def plan_paths(start, end, f1, f2) -> list:
    global PATHS
    p_lead  = create_one_path( np.array(start), np.array(end)   )
    p_f1    = create_one_path(np.array(f1), np.array(start)) + p_lead[1:-config.FOLLOW_DIST_PIXEL]
    p_f2    = create_one_path(np.array(f2), np.array(f1)) + p_f1[1:-config.FOLLOW_DIST_PIXEL]
    PATHS   = [p_lead, p_f1, p_f2] 
    return PATHS

# End Task 4
##########################################################################################################



##########################################################################################################
"""
    Task 5: Extract Next Waypoints
    Lead: Joshua
"""
def get_next_waypoints(bot_id, bot_pos):
    waypoints_arr = np.array(PATHS[bot_id])
    bot_pos_arr = np.array(bot_pos)
    distances = np.linalg.norm(waypoints_arr - bot_pos_arr, axis=1)
    closest_index = np.argmin(distances)
    
    if (len(PATHS[bot_id]) - 1) - closest_index > 5 :
        return PATHS[bot_id][closest_index: closest_index + 5]
    else:
        return PATHS[bot_id][closest_index:]

# End Task 5
##########################################################################################################

##########################################################################################################
"""
    Task 6: Send Waypoints
    Lead: Owen
"""

def template6():
    for bot_id in range(3):
        bot_pos = [1,1] # fill in with Zohaib's functions or variables
        next_waypoints = get_next_waypoints(bot_id, bot_pos)        # returns the next 5 waypoints for bot_id

# End Task 6
##########################################################################################################


##########################################################################################################
"""
    Task 7: Create a very important task for an example
    Lead: Joshua
"""

def very_important_task():
    img = Image.open("IMG_20230306.jpg")
    img.show()

# End Task 7
##########################################################################################################


##########################################################################################################
# Main Loop
if __name__ == "__main__":
    very_important_task()