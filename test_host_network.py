from shared.network import JetsonNetwork
import time

# Main
network = JetsonNetwork()
network.start(num_devices=1)

while (True):
    network.send(0, [(1,1), (2,2)])
    time.sleep(1)
