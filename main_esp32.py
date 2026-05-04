from shared import network as esp_network
from esp32 import imu, motor_driver
from esp32.ultrasonic import Ultrasonic
import config
import robot
import time

# ==========================================
# IMPORTANT: SET THIS BEFORE FLASHING
# 0 = Leader, 1 = Follower 1, 2 = Follower 2
# ==========================================
R_ID = 0       # config the id of the robot

if (R_ID == 0):
    START_POINT = config.R0_START
    END_POINT   = config.R0_END
elif (R_ID == 1):
    START_POINT = config.R0_START
    END_POINT   = config.R0_END
elif (R_ID == 2):
    START_POINT = config.R0_START
    END_POINT   = config.R0_END

if __name__ == "__main__":
    # Fetch classes
    DRIVER = motor_driver.TB6612FNG()
    NET = esp_network.Esp32Network(R_ID)
    ROBOT = robot.robot(DRIVER, START_POINT, END_POINT)
    
    # Initialize Ultrasonic Sensor (Pins 33 & 32 as per ultrasonic.py)
    SONAR = Ultrasonic(trig_pin=33, echo_pin=32)

    # initialize
    NET.start()
    last_msg = time.time()

    print(f"Robot {R_ID} Started. Connecting to {config.PI_IP}...")

    # control loop
    while (True):
        # 1. OBSTACLE AVOIDANCE
        distance = SONAR.get_distance_cm()
        if distance is not None and distance < 15.0:
            # Obstacle within 15 cm -> Stop immediately
            print(f"Obstacle detected at {distance:.1f}cm! Stopping.")
            ROBOT.emergency_stop()
            time.sleep(0.1)
            continue # Skip waypoint processing until clear

        # 2. PROCESS WAYPOINTS
        # Check if waypoints were sent to the ESP
        if (NET.poll()):
            [yaw, _ ], curr_pos, goal_pos = NET.recv().data
            ROBOT.run(curr_pos, goal_pos, yaw)
            last_msg = time.time()

        # otherwise if too much time between msgs -> force stop
        elif (time.time() - last_msg) >= config.NET_TIMEOUT:
            # print(f"ERROR. Network timeout") # Muted to prevent spam
            ROBOT.emergency_stop()