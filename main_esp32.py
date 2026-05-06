from shared import network as esp_network
from esp32 import imu, motor_driver
import config
import robot
import time

R_ID = 0       # config the id of the robot

if (R_ID == 0):
    START_POINT = config.R0_START
    END_POINT   = config.R0_END
elif (R_ID == 1):
    START_POINT = config.R1_START
    END_POINT   = config.R1_END
else (R_ID == 2):
    START_POINT = config.R2_START
    END_POINT   = config.R2_END

if __name__ == "__main__":
    # Fetch classes
    DRIVER = motor_driver.TB6612FNG()
    NET = esp_network.Esp32Network()
    ROBOT = robot.robot(DRIVER, START_POINT, END_POINT)

    # initialize
    NET.start(config.PI_IP, 60007)
    last_msg = time.time()

    # control loop
    while (True):
        # Check if waypoints were sent to the ESP
        if (NET.poll()):
            (yaw, _ ), curr_pos, goal_pos = NET.recv().data
            ROBOT.run(curr_pos, goal_pos, yaw)
            last_msg = time.time()

        # otherwise if too much time between msgs -> force stop
        elif ((time.time() - last_msg) >= config.NET_TIMEOUT):
            print(f"ERROR. Network timeout")
            NET.close()
            ROBOT.emergency_stop()
