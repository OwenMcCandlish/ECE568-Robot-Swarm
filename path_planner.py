# Imports
import numpy as np
import math
import config

# Globals
PATHS = [ [], [], [] ]          # Global Variable for accessing the planned paths for each bot


def create_one_path(start, end):
    min_num_points = np.linalg.norm(end - start).astype(int) + 1
    line = np.linspace(start, end, min_num_points)
    line_rounded = np.round(line).astype(int)
    line_unique = np.unique(line_rounded, axis=0)
    return line_unique.tolist()

def plan_paths(start, end, f1, f2) -> list:
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
    global PATHS
    p_lead  = create_one_path( np.array(start), np.array(end)   )
    p_f1    = create_one_path(np.array(f1), np.array(start)) + p_lead[1:-config.FOLLOW_DIST_PIXEL]
    p_f2    = create_one_path(np.array(f2), np.array(f1)) + p_f1[1:-config.FOLLOW_DIST_PIXEL]
    PATHS   = [p_lead, p_f1, p_f2]
    return PATHS

def get_next_waypoints(bot_id, bot_pos):
    """
        Task 5: Extract Next Waypoints
        Lead: Joshua
    """
    waypoints_arr = np.array(PATHS[bot_id])
    bot_pos_arr = np.array(bot_pos)
    
    if len(waypoints_arr) == 0:
        return [bot_pos] # Fallback if path is empty
        
    distances = np.linalg.norm(waypoints_arr - bot_pos_arr, axis=1)
    closest_index = np.argmin(distances)

    if (len(PATHS[bot_id]) - 1) - closest_index > 5 :
        return PATHS[bot_id][closest_index: closest_index + 5]
    else:
        return PATHS[bot_id][closest_index:]

def update_dynamic_paths(cur_locs, cur_headings):
    """
    Dynamically recalculates the paths for the followers based on the actual
    real-time position of the leader and follower 1.
    """
    global PATHS
    if len(cur_locs) < 3 or len(cur_headings) < 3:
        return
        
    leader_loc = cur_locs[0]
    leader_heading = math.radians(cur_headings[0])
    
    f1_loc = cur_locs[1]
    f1_heading = math.radians(cur_headings[1])
    
    # F1 target is behind leader
    dx1 = -config.FOLLOW_DIST_PIXEL * math.cos(leader_heading)
    dy1 = -config.FOLLOW_DIST_PIXEL * math.sin(leader_heading)
    f1_target = [leader_loc[0] + dx1, leader_loc[1] + dy1]
    
    # F2 target is behind F1
    dx2 = -config.FOLLOW_DIST_PIXEL * math.cos(f1_heading)
    dy2 = -config.FOLLOW_DIST_PIXEL * math.sin(f1_heading)
    f2_target = [f1_loc[0] + dx2, f1_loc[1] + dy2]
    
    # Update PATHS[1] and PATHS[2] to just be a straight line from their current loc to target
    PATHS[1] = create_one_path(np.array(f1_loc), np.array(f1_target))
    PATHS[2] = create_one_path(np.array(cur_locs[2]), np.array(f2_target))

