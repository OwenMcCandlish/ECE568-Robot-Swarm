##################################################################################
# Imports
from time import sleep
import math
from pid import pid_speed_controller

##################################################################################
# Globals
PID_CONTROLLER = pid_speed_controller()


##################################################################################
"""
    Task 1: Initialize Hardware
    Lead:
"""

def template1():
    print(f"Create functions here")

# End Task 1
##################################################################################



##################################################################################
"""
    Task 2: Initialize IMU/Compass
    Lead:
"""

def template2():
    print(f"Create functions here")

# End Task 2
##################################################################################


##################################################################################
"""
    Task 2: Initialize PID Controller
    Lead: Joshua Vandaveer
"""

def init_pid(goal, start):
    pid_speed_controller.goal = goal
    pid_speed_controller.starting_pos = start

# End Task 2
##################################################################################



##################################################################################
"""
    Task 3: Initialize Network
    Lead: Owen
"""

def template3():
    print(f"Create functions here")

# End Task 3
##################################################################################


##################################################################################
"""
    Task 4: Receive Waypoints
    Lead: Owen
"""

def template4():
    print(f"Create functions here")

# End Task 4
##################################################################################


##################################################################################
"""
    Task 4: Convert Index to Position
    Lead: Zohaib
"""

def template4():
    print(f"Create functions here")

# End Task 4
##################################################################################



##################################################################################
"""
    Task: Calculate Headings
    Lead: Joshua
    Requires:
                pos         <list>          x,y coordinates of the bot's current position
                goal        <list>          x,y coordinates of the bot's goal endpoints
                orientation <float>         orientation in radians
    Returns:    heading     <list>          v, w headings of the bot
"""

def set_speed_limits(v, w):
    # CASE: zero linear velocity
    if (v == 0.0):
        if (w < -config.MAX_W):
            w = - config.MAX_W
        elif (w > config.MAX_W):
            w = config.MAX_W
        return v, w

    if (w < - config.MAX_W):
        r = v / w
        w = -config.MAX_W
        v = r * w
    if (w > config.MAX_W):
        r = v / w               # keep inside condition to avoid division by zero
        w = config.MAX_W
        v = r * w
    return v, w

def calculate_headings(pos, goal, orientation):
    dx = pos[0] - goal[0]
    dy = pos[1] - goal[1]
    l = math.sqrt( (dx**2) + (dy**2) )

    theta_line = math.atan2(dy,dx)
    theta_offset = theta_line - orientation

    if ( abs(l * math.sin(theta_offset)) < 0.01 ):
        w = 0.0
    else:
        w = -(2 * v_limit * math.sin(theta_offset)) / l

    v_limit = pid_speed_controller.pid_calculate(pos)
    v, w = self.set_speed_limits(v_limit, w)
    return v, w


# End Task 4
##################################################################################



##################################################################################
"""
    Task 6: Calculate Actuator Duty Cycles
    Lead:
"""

def template6():
    print(f"Create functions here")

# End Task 5
##################################################################################


##################################################################################
"""
    Task 7: Create a very important task for an example
    Lead: Joshua
"""

def very_important_task():
    sleep(1)
    print("I walked past a homeless guy the other day.")
    sleep(3)
    print("His sign said: \"One day, this could be you\"")
    sleep(4)
    print("so...")
    sleep(2)
    print("I put my money back in my pocket")
    sleep(3)
    print("Just in case he's right")
    sleep(2)

# End Task 7
##################################################################################



##################################################################################
# Main Loop

if __name__ == "__main__":
    very_important_task()
