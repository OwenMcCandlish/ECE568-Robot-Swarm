##########################################################################################################
# Imports
import config
import time

##########################################################################################################
# PID Controller for the Velocity Calculations
# init arguments: goal_pos [x,y], starting_pos [x, y]
class pid_speed_controller():
    def __init__(self, start, end):
        kp = config.MAX_V / config.MAX_E
        kd = 0.2
        ki = (kp**2) / (4*(1+kd))
        self.PID = [kp, ki, kd]

        self.goal_pos = end

        # previous position and the time it was taken (for velocity calculations)
        self.prev_position = start
        self.prev_time_stamp = time.time()
        self.error_sum = 0

        # error queue for tracking the sum of errors
        self.error_dq = deque(maxlen=20)
        self.v_max = config.MAX_V

    #########################################################################################################
    # CALCULATE_DISTANCE:
    def distance(self, point1, point2):
        delta_x = (point1[0] - point2[0])
        delta_y = (point1[1] - point2[1])
        dist = math.sqrt(delta_x**2 + delta_y**2)
        return dist

    #########################################################################################################
    # PID_CALCULATE: return V
    def pid_calculate(self, pos):
        e = self.distance(self.goal_pos, pos)
        if math.isinf(e):
            e = config.BOT_LENGTH_CM * 2
        e_1 = (self.distance(pos, self.prev_position)) / (time.time() - self.prev_time_stamp) # -dx/dt
        V = self.calculate_velocity(e, e_1)
        self.update(e, pos)
        return V

    #########################################################################################################
    # UPDATE: update the time, error sum, and the previous position (for current Velocity calculation)
    def update(self, e, pos):
        self.error_dq.append(e)                             # add the error to the queue (pop off old values)
        self.error_sum = sum(self.error_dq)                 # error is the sum of 10 most recent errors
        self.prev_position = pos
        self.prev_time_stamp = time.time()

    #########################################################################################################
    # CALCULATE_VELOCITY: return velocity
    def calculate_velocity(self, e, e_1):
        if abs(e) <= 0.02:           # are we accurate to within 2 cm?
            return 0.0               # then just say screw it, that's good enough

        # get the current error sum
        e_sum = self.error_sum

        # use the PID parameters to get the current V value
        V = (e * self.PID[0]) + (e_sum * self.PID[1]) + (e_1 * self.PID[2])

        # limit the V appropriately
        if V > self.v_max:
            V = self.v_max
        if V < - self.v_max:
            V = - self.v_max
        return V