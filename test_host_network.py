from shared.network import JetsonNetwork
import sleep

# Main
network = JetsonNetwork()
network.start()

while (True):
    network.send(0, [(1,1), (2,2)])
    time.sleep(1)
