# ==========================================
# ESP32 SIMPLE CORNER-TO-CORNER ROBOT CODE
# Save this file on ESP32 as main.py
# ==========================================

R_ID = 0

import math
import time
import socket
import json
from machine import Pin, PWM


# =========================
# WiFi / Pi Network Config
# =========================

PI_IP = "192.168.137.164"
WIFI_SSID = "owen"
WIFI_PWD = "190154om03"

PI_PORT = 5005
NET_TIMEOUT = 2.0


# =========================
# Motor Pins
# =========================

AIN1 = 13
AIN2 = 14
PWMA = 26

BIN1 = 12
BIN2 = 27
PWMB = 25

STBY = 33


# =========================
# Motor Driver
# =========================

class Motor:
    def __init__(self, in1_pin, in2_pin, pwm_pin, standby_pin, freq=5000):
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin), freq=freq)

        self.stby = Pin(standby_pin, Pin.OUT)
        self.stby.value(1)

    def run(self, speed):
        """
        speed: -100 to 100
        Positive = forward
        Negative = reverse
        """

        speed = max(-100, min(100, speed))

        if abs(speed) < 1:
            self.stop()
            return

        MIN_DUTY = 700
        MAX_DUTY = 1023

        duty = int(MIN_DUTY + (abs(speed) / 100.0) * (MAX_DUTY - MIN_DUTY))
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


class TB6612FNG:
    def __init__(self):
        self.stby = Pin(STBY, Pin.OUT)
        self.stby.value(1)

        self.motor_left = Motor(AIN1, AIN2, PWMA, STBY)
        self.motor_right = Motor(BIN1, BIN2, PWMB, STBY)

    def wake(self):
        self.stby.value(1)

    def set_speeds(self, right_speed, left_speed):
        self.wake()
        self.motor_right.run(right_speed)
        self.motor_left.run(left_speed)

    def stop(self):
        self.motor_right.stop()
        self.motor_left.stop()


# =========================
# Network
# =========================

class Message:
    def __init__(self, data):
        self.data = data


class Esp32Network:
    def __init__(self, r_id):
        self.r_id = r_id

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.setblocking(False)

        self.msg_queue = []
        self.last_hello = 0

    def start(self):
        self.send_hello()

    def send_hello(self):
        try:
            msg = json.dumps({
                "r_id": self.r_id,
                "hello": True
            }).encode("utf-8")

            self.sock.sendto(msg, (PI_IP, PI_PORT))
        except Exception:
            pass

    def poll(self):
        has_new = False

        try:
            while True:
                data, addr = self.sock.recvfrom(2048)
                msg = json.loads(data.decode("utf-8"))

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


# =========================
# Simple Robot Controller
# =========================

class Robot:
    def __init__(self, driver):
        self.driver = driver

        # Main tuning values
        self.FORWARD_SPEED = 28
        self.TURN_SPEED = 25
        self.ANGLE_TOLERANCE_DEG = 20.0

        # Stop when within this distance of target point
        self.GOAL_RADIUS_CM = 6.0

        # Flip this if robot rotates away from target
        self.TURN_SIGN = -1

    def normalize_angle_deg(self, angle):
        while angle > 180:
            angle -= 360

        while angle < -180:
            angle += 360

        return angle

    def run(self, curr_pos, goal_pos, orientation_deg):
        dx = goal_pos[0] - curr_pos[0]
        dy = goal_pos[1] - curr_pos[1]

        distance = math.sqrt(dx * dx + dy * dy)

        if distance <= self.GOAL_RADIUS_CM:
            print("STOP: target reached",
                  "pos=", curr_pos,
                  "goal=", goal_pos,
                  "dist=", distance)
            self.driver.stop()
            return

        desired_heading = math.degrees(math.atan2(dy, dx))
        heading_error = self.normalize_angle_deg(desired_heading - orientation_deg)

        if abs(heading_error) > self.ANGLE_TOLERANCE_DEG:
            if heading_error > 0:
                turn = self.TURN_SPEED * self.TURN_SIGN
            else:
                turn = -self.TURN_SPEED * self.TURN_SIGN

            right_cmd = turn
            left_cmd = -turn

            print("TURN:",
                  "pos=", curr_pos,
                  "goal=", goal_pos,
                  "yaw=", orientation_deg,
                  "desired=", desired_heading,
                  "err=", heading_error,
                  "R=", right_cmd,
                  "L=", left_cmd)

            self.driver.set_speeds(right_cmd, left_cmd)
            return

        print("STRAIGHT:",
              "pos=", curr_pos,
              "goal=", goal_pos,
              "yaw=", orientation_deg,
              "dist=", distance)

        self.driver.set_speeds(self.FORWARD_SPEED, self.FORWARD_SPEED)

    def emergency_stop(self):
        self.driver.stop()


# =========================
# WiFi
# =========================

def connect_wifi():
    import network

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        print("Connecting to WiFi:", WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PWD)

        while not wlan.isconnected():
            time.sleep(0.5)
            print(".", end="")

        print()

    print("WiFi Connected!", wlan.ifconfig())


# =========================
# Main
# =========================

if __name__ == "__main__":
    connect_wifi()

    driver = TB6612FNG()
    net = Esp32Network(R_ID)
    robot = Robot(driver)

    net.start()
    last_msg = time.time()

    print("Robot", R_ID, "started. Waiting for Pi commands at", PI_IP)

    while True:
        if net.poll():
            msg = net.recv().data

            try:
                [yaw, _], curr_pos, goal_pos = msg
                robot.run(curr_pos, goal_pos, yaw)
                last_msg = time.time()

            except Exception as e:
                print("Bad command:", msg, e)
                robot.emergency_stop()

        elif time.time() - last_msg >= NET_TIMEOUT:
            robot.emergency_stop()