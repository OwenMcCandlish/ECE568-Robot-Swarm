import time

from jetson.vision import Vision
from shared.network import JetsonNetwork
import config
import jetson.path_planner as path_planner

def main():
    network = JetsonNetwork()
    vision = Vision(show_feed=True)

    # Start Network
    network.start(config.NUM_DEVICES)

    # Create Paths
    leader = vision.locate_robots()[0]
    follower1 = (0,0)
    follower2 = (0,0)
    path_planner.plan_paths(leader, config.END_POINT, follower1)

    while (True):
        # get bot positions
        cur_locs, cur_headings = vision.locate_robots()

        # send each bot its location and next point
        for i in range(len(cur_locs)):
            cur_loc: tuple[int, int] = cur_locs[i]
            cur_heading: int = cur_headings[i]

            desired_path = path_planner.get_next_waypoints(i, cur_loc)
            next_loc = desired_path[-1] # send the last point in the desired path

            network.send(i, data=[(cur_heading, 0), cur_loc, next_loc])
            # print(f"Corners: {}")
            # print(f"Heading: {cur_heading}, Location: {cur_loc}")
        time.sleep(0.10) # Jetson refresh rate

if __name__ == "__main__":
    main()

