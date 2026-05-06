from shared import network as esp_network
from esp32_lib import imu, motor_driver, robot
import config
import time

R_ID = 0       # config the id of the robot

if __name__ == "__main__":
    # Fetch classes
    DRIVER = motor_driver.TB6612FNG()
    NET = esp_network.Esp32Network()

    # initialize
    NET.start(config.PI_IP, 60007)

    # Get initial position
    while(not NET.poll()):
        time.sleep_ms(50)
    (yaw, _ ), curr_pos, goal_pos = NET.recv().data

    ROBOT = robot.robot(DRIVER, curr_pos, goal_pos)

    # control loop
    last_msg = time.time()
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
        time.sleep_ms(10)
