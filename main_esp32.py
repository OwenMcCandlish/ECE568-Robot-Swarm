from shared import network as esp_network
from esp32_lib import imu, motor_driver, robot
import config
import time
import network


R_ID = 2       # config the id of the robot

def connect_to_wifi():
    ssid = config.WIFI_SSID
    pwd = config.WIFI_PWD
    nic = network.WLAN(network.STA_IF)
    nic.active(True)
    if (not nic.isconnected()):
        nic.connect(ssid, pwd)
    while not nic.isconnected():
        time.sleep_ms(10)
    ip_addr = nic.ifconfig()[0]
    print("Connected to {}".format(ssid))
    print("IP Address: {}\n".format(ip_addr))
    return ip_addr

def main():
    connect_to_wifi()

    # Fetch classes
    DRIVER = motor_driver.TB6612FNG()
    NET = esp_network.Esp32Network(R_ID)

    # initialize network
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

if __name__ == "__main__":
    main()
