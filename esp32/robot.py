##################################################################################
# Imports
from time import sleep
import math
from pid import pid_speed_controller
import config

##################################################################################
# Globals
PID_CONTROLLER = pid_speed_controller()

class robot():
    def __init__(self, driver, start, end):
        self.driver = driver
        self.pos = start
        self.pid = pid_speed_controller(start, end)

    def set_w_speed_limit(self, v, w):
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

    def calculate_headings(self, pos, goal, orientation):
        dx = pos[0] - goal[0]
        dy = pos[1] - goal[1]
        l = math.sqrt( (dx**2) + (dy**2) )

        theta_line = math.atan2(dy,dx)
        theta_offset = theta_line - orientation

        v_limit = self.pid.pid_calculate(pos)

        if ( abs(l * math.sin(theta_offset)) < 0.01 ):
            w = 0.0
        else:
            w = -(2 * v_limit * math.sin(theta_offset)) / l

        v, w = self.set_w_speed_limit(v_limit, w)
        return v, w

    def calc_v_goals(self, v,w):
        v_right = ( (w * config.BOT_WIDTH_CM) + 2 * v) / 2
        v_left = 2 * v - v_right
        return v_right, v_left

    def calculate_actuator_speeds(self, v,w):
        vr, vl = self.calc_v_goals(v,w)
        dc_r = config.DC_SCALE * vr
        dc_l = config.DC_SCALE *vl
        return dc_r, dc_l

    def run(self, curr_pos, goal_pos, orientation):
        v, w = self.calculate_headings(curr_pos, goal_pos, orientation)
        dc_r, dc_l = self.calculate_actuator_speeds(v,w)
        self.driver.set_speeds(dc_r, dc_l)

    def emergency_stop(self):
        self.driver.stop()
