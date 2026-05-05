import numpy as np
import time

from pi_apriltag_tracker import Vision
from swarm_network import JetsonNetwork
import config
import path_planner

def main():
    network = JetsonNetwork()
    vision = Vision()

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

        # Dynamically update follower target paths based on real-time movements
        path_planner.update_dynamic_paths(cur_locs, cur_headings)

        # send each bot its location and next point
        for i in range(1):
            cur_loc: tuple[int, int] = cur_locs[i]
            cur_heading: int = cur_headings[i]

            desired_path = path_planner.get_next_waypoints(i, cur_loc)
            next_loc = desired_path[-1] # send the last point in the desired path

            network.send(i, data=[[cur_heading, 0], cur_loc, next_loc])
        time.sleep(0.20) # Jetson refresh rate

if __name__ == "__main__":
    main()

