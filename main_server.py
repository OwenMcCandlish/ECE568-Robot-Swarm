import numpy as np
import time

from pi_apriltag_tracker import Vision
from swarm_network import JetsonNetwork
import config
import path_planner

def main():
    network = JetsonNetwork()
    vision = Vision(show_feed=True)

    # Start Network
    network.start(config.NUM_DEVICES)

    # Create leader-only path
    while True:
        cur_locs, cur_headings = vision.locate_robots()

        if len(cur_locs) >= 1 and cur_locs[0] != (0, 0):
            leader = cur_locs[0]
            break

        print("Waiting for leader tag ID 0...")
        time.sleep(0.2)

    path_planner.PATHS[0] = path_planner.create_one_path(
        np.array(leader),
        np.array(config.END_POINT)
    )

    while (True):
        # get bot positions
        cur_locs, cur_headings = vision.locate_robots()

        if len(cur_locs) < 1 or cur_locs[0] == (0, 0):
            print("Leader tag lost. Not sending command.")
            time.sleep(0.2)
            continue

        # Dynamically update follower target paths based on real-time movements
        #path_planner.update_dynamic_paths(cur_locs, cur_headings)

        # send each bot its location and next point
        for i in range(1):
            cur_loc: tuple[int, int] = cur_locs[i]
            cur_heading: int = cur_headings[i]

            x, y = cur_loc
            if x < 0 or x > 82 or y < 0 or y > 82:
                print(f"[SAFETY] Robot out of arena at {cur_loc}. Sending stop.")
                next_loc = cur_loc
            else:
                next_loc = config.END_POINT

            sent = network.send(i, data=[[cur_heading, 0], cur_loc, next_loc])
            print(f"[SEND] Robot {i} sent={sent} cur={cur_loc} heading={cur_heading} next={next_loc} final={config.END_POINT}")
        time.sleep(0.20) # Jetson refresh rate

if __name__ == "__main__":
    main()

