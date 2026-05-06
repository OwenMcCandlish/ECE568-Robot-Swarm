# ==========================================
# ESP32 STANDALONE SWARM ROBOT CODE
# ==========================================
# IMPORTANT: SET THIS BEFORE FLASHING
# 0 = Leader, 1 = Follower 1, 2 = Follower 2
# ==========================================
R_ID = 0

import math
import time
import socket
import json
from machine import Pin, time_pulse_us, PWM

# MicroPython collections fallback
try:
    from collections import deque
except ImportError:
    class deque:
        def __init__(self, iterable=(), maxlen=None):
            self.q = list(iterable)
            self.maxlen = maxlen
        def append(self, item):
            self.q.append(item)
            if self.maxlen is not None and len(self.q) > self.maxlen:
                self.q.pop(0)
        def __iter__(self):
            return iter(self.q)

# ----------------- CONFIGURATION -----------------
PIXEL_LENGTH                = 2                                         
BOT_LENGTH_CM               = 20                                        
BOT_LENGTH_PIXEL            = BOT_LENGTH_CM // PIXEL_LENGTH             
BOT_WIDTH_CM                = 10                                        
BOT_WIDTH_PIXEL             = BOT_WIDTH_CM // PIXEL_LENGTH              
FOLLOW_DIST_CM              = 10                                        
FOLLOW_DIST_PIXEL           = FOLLOW_DIST_CM // PIXEL_LENGTH            

MAX_V                       = 10                                        
MAX_W                       = math.pi / 4                               

LOOKAHEAD_LEN               = 5                                         
INTERVAL                    = 0.1                                       
MAX_E                       = 20 * MAX_V * INTERVAL                     

NET_TIMEOUT                 = 2                                         
NUM_DEVICES                 = 3
END_POINT                   = (500, 350)

R0_START = [10,10]
R0_END   = [90,90]

R1_START = [5,5]
R1_END   = [85,85]

R2_START = [0,0]
R2_END   = [80,80]

DC_SCALE = 3                    

PI_IP = "172.20.10.2"
WIFI_SSID = "ZOHAIBSINTERNET"
WIFI_PWD = "Zohaibisbest"

# ----------------- NETWORK -----------------
class Message:
    def __init__(self, data):
        self.data = data

class Esp32Network:
    def __init__(self, r_id):
        self.r_id = r_id
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 0))
        self.sock.setblocking(False)
        self.pi_ip = PI_IP
        self.pi_port = 5005
        self.msg_queue = []
        self.last_hello = 0

    def start(self):
        self.send_hello()

    def send_hello(self):
        try:
            msg = json.dumps({"r_id": self.r_id}).encode('utf-8')
            self.sock.sendto(msg, (self.pi_ip, self.pi_port))
        except Exception as e:
            pass

    def poll(self):
        has_new = False
        try:
            while True:
                data, addr = self.sock.recvfrom(1024)
                msg = json.loads(data.decode('utf-8'))
                if "data" in msg:
                    self.msg_queue.append(Message(msg["data"]))
                    has_new = True
        except OSError:
            pass
        
        if time.time() - self.last_hello > 2:
            self.send_hello()
            self.last_hello = time.time()

        return has_new

    def recv(self):
        if self.msg_queue:
            return self.msg_queue.pop(-1)
        return Message([])

# ----------------- ULTRASONIC -----------------
class Ultrasonic:
    SOUND_SPEED_CM_US = 0.0343

    def __init__(self, trig_pin=33, echo_pin=32, timeout_us=30000):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.timeout_us = timeout_us
        self.trig.off()

    def get_distance_cm(self):
        self.trig.off()
        time.sleep_us(2)

        self.trig.on()
        time.sleep_us(10)
        self.trig.off()

        duration = time_pulse_us(self.echo, 1, self.timeout_us)

        if duration < 0:
            return None

        return (duration * self.SOUND_SPEED_CM_US) / 2

# ----------------- MOTOR DRIVER -----------------
AIN1 = 13
AIN2 = 14
PWMA = 26

BIN1 = 12
BIN2 = 27
PWMB = 25

STBY = 33


class Motor:
    def __init__(self, in1_pin, in2_pin, pwm_pin, standby_pin, freq=5000):
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin), freq=freq)
        self.stby = Pin(standby_pin, Pin.OUT)
        self.stby.value(1)

    def run(self, speed):
        """
        speed is from -100 to 100 percent.
        Converts to a strong TB6612 PWM duty.
        Your working motor test used duty=800, so we enforce a minimum moving duty.
        """
        speed = max(-100, min(100, speed))

        if abs(speed) < 1:
            self.stop()
            return

        MIN_DUTY = 750
        MAX_DUTY = 1023

        duty = int(MIN_DUTY + (abs(speed) / 100) * (MAX_DUTY - MIN_DUTY))
        duty = min(duty, MAX_DUTY)

        if speed > 0:
            self.in1.value(1)
            self.in2.value(0)
        else:
            self.in1.value(0)
            self.in2.value(1)

        self.pwm.duty(duty)

    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty(0)

    def brake(self):
        self.in1.value(1)
        self.in2.value(1)
        self.pwm.duty(1023)

    def deinit(self):
        self.stop()
        self.pwm.deinit()


class TB6612FNG:
    def __init__(self):
        self.stby = Pin(STBY, Pin.OUT)
        self.stby.value(1)

        # Based on your working motor test
        self.motor_left = Motor(AIN1, AIN2, PWMA, STBY)
        self.motor_right = Motor(BIN1, BIN2, PWMB, STBY)

    def wake(self):
        self.stby.value(1)

    def standby(self):
        self.stby.value(0)

    def set_speeds(self, right_speed, left_speed):
        self.wake()
        self.motor_right.run(right_speed)
        self.motor_left.run(left_speed)

    def stop(self):
        self.motor_right.stop()
        self.motor_left.stop()

    def brake(self):
        self.motor_right.brake()
        self.motor_left.brake()

    def deinit(self):
        self.stop()
        self.standby()
        self.motor_right.deinit()
        self.motor_left.deinit()

# ----------------- PID -----------------
class pid_speed_controller():
    def __init__(self, start, end):
        kp = MAX_V / MAX_E
        kd = 0.2
        ki = (kp**2) / (4*(1+kd))
        self.PID = [kp, ki, kd]

        self.goal_pos = end

        self.prev_position = start
        self.prev_time_stamp = time.time()
        self.error_sum = 0

        self.error_dq = []
        self.v_max = MAX_V

    def distance(self, point1, point2):
        delta_x = (point1[0] - point2[0])
        delta_y = (point1[1] - point2[1])
        dist = math.sqrt(delta_x**2 + delta_y**2)
        return dist

    def pid_calculate(self, pos):
        e = self.distance(self.goal_pos, pos)
        if math.isinf(e):
            e = BOT_LENGTH_CM * 2
        e_1 = (self.distance(pos, self.prev_position)) / (max(time.time() - self.prev_time_stamp, 0.001))
        V = self.calculate_velocity(e, e_1)
        self.update(e, pos)
        return V

    def update(self, e, pos):
        self.error_dq.append(e)
        if len(self.error_dq) > 20:
            self.error_dq.pop(0)
        self.error_sum = sum(self.error_dq)
        self.prev_position = pos
        self.prev_time_stamp = time.time()

    def calculate_velocity(self, e, e_1):
        if abs(e) <= 0.02:
            return 0.0

        e_sum = self.error_sum
        V = (e * self.PID[0]) + (e_sum * self.PID[1]) + (e_1 * self.PID[2])

        if V > self.v_max:
            V = self.v_max
        if V < - self.v_max:
            V = - self.v_max
        return V

# ----------------- ROBOT -----------------
class robot():
    def __init__(self, driver, start, end):
        self.driver = driver
        self.pos = start
        self.pid = pid_speed_controller(start, end)

    def set_w_speed_limit(self, v, w):
        if (v == 0.0):
            if (w < -MAX_W):
                w = - MAX_W
            elif (w > MAX_W):
                w = MAX_W
            return v, w

        if (w < - MAX_W):
            r = v / w
            w = -MAX_W
            v = r * w
        if (w > MAX_W):
            r = v / w               
            w = MAX_W
            v = r * w
        return v, w

    def calculate_headings(self, pos, goal, orientation):
        orientation = math.radians(orientation)

        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]
        l = math.sqrt((dx ** 2) + (dy ** 2))

        if l <= 2.0:
            return 0.0, 0.0

        theta_line = math.atan2(dy, dx)
        theta_offset = theta_line - orientation

        # Normalize to [-pi, pi]
        while theta_offset > math.pi:
            theta_offset -= 2 * math.pi
        while theta_offset < -math.pi:
            theta_offset += 2 * math.pi

        # If badly misaligned, rotate in place first.
        if abs(theta_offset) > math.radians(35):
            v = 0.0

            if theta_offset > 0:
                w = -MAX_W
            else:
                w = MAX_W

            return v, w

        # If roughly aligned, move forward and steer.
        v_limit = self.pid.pid_calculate(pos)

        # IMPORTANT: no negative sign here
        w = -(2 * v_limit * math.sin(theta_offset)) / max(l, 0.001)

        v, w = self.set_w_speed_limit(v_limit, w)
        return v, w

    def calc_v_goals(self, v,w):
        v_right = ( (w * BOT_WIDTH_CM) + 2 * v) / 2
        v_left = 2 * v - v_right
        return v_right, v_left

    def calculate_actuator_speeds(self, v,w):
        vr, vl = self.calc_v_goals(v,w)
        dc_r = DC_SCALE * vr
        dc_l = DC_SCALE *vl
        return dc_r, dc_l

    def run(self, curr_pos, goal_pos, orientation):
        self.pid.goal_pos = goal_pos
        v, w = self.calculate_headings(curr_pos, goal_pos, orientation)
        dc_r, dc_l = self.calculate_actuator_speeds(v, w)

        print("CTRL:", "pos=", curr_pos, "goal=", goal_pos, "yaw=", orientation,
            "v=", v, "w=", w, "R=", dc_r, "L=", dc_l)

        self.driver.set_speeds(dc_r, dc_l)

    def emergency_stop(self):
        self.driver.stop()


# ----------------- MAIN LOOP -----------------
if R_ID == 0:
    START_POINT = R0_START
    END_POINT   = R0_END
elif R_ID == 1:
    START_POINT = R0_START
    END_POINT   = R0_END
elif R_ID == 2:
    START_POINT = R0_START
    END_POINT   = R0_END

def connect_wifi():
    import network
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print(f"Connecting to WiFi: {WIFI_SSID}...")
        wlan.connect(WIFI_SSID, WIFI_PWD)
        while not wlan.isconnected():
            time.sleep(0.5)
            print(".", end="")
        print()
    print("WiFi Connected!", wlan.ifconfig())

if __name__ == "__main__":
    connect_wifi()
    DRIVER = TB6612FNG()
    NET = Esp32Network(R_ID)
    ROBOT = robot(DRIVER, START_POINT, END_POINT)
    
   # SONAR = Ultrasonic(trig_pin=4, echo_pin=32)

    NET.start()
    last_msg = time.time()

    print(f"Robot {R_ID} Started. Connecting to {PI_IP}...")

    while (True):
        if (NET.poll()):
            [yaw, _ ], curr_pos, goal_pos = NET.recv().data
            ROBOT.run(curr_pos, goal_pos, yaw)
            last_msg = time.time()

        elif (time.time() - last_msg) >= NET_TIMEOUT:
            ROBOT.emergency_stop()
