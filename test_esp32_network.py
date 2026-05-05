from shared.network import Esp32Network, Packet
from sleep import sleep_ms

# Main
network = Esp32Network(0)
network.start()
while (True):
    if (network.poll()):
        packet = network.recv()
        print(packet.data)

