from shared.network import Esp32Network, Packet
import time

import network

def connect_to_wifi():
    ssid = "owen"
    pwd = "190154om03"
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

# Main
connect_to_wifi()
network = Esp32Network(0)
network.start(ip = '172.20.10.1', port = 60007)
while (True):
    if (network.poll()):
        packet = network.recv()
        print(packet.data)

    time.sleep_ms(100)

