import numpy as np
import time

from pi_apriltag_tracker import Vision
from swarm_network import JetsonNetwork
import config
import path_planner

WAYPOINT_RADIUS_CM = 5.0
FINAL_RADIUS_CM = 6.0

def dist(a, b):
    return float(np.linalg.norm(np.array(a, dtype=float) - np.array(b, dtype=float)))

def main():
    network = JetsonNetwork()
    vision = Vision(show_feed=True)

    network.start(config.NUM_DEVICES)

    while True:
        cur_locs, cur_headings = vision.locate_robots()

        if len(cur_locs) >= 1 and cur_locs[0] != (0, 0):
            leader = cur_locs[0]
            print(f"[VISION] Leader detected at {leader}, heading={cur_headings[0]}")
            break

        print("Waiting for leader tag ID 0...")
        time.sleep(0.2)

    arena = config.ARENA_SIZE_CM
    margin = 10.0
    path_planner.PATHS[0] = [
        leader,
        (margin, margin),
        (margin, arena - margin),
        (arena - margin, arena - margin),
        (arena - margin, margin),
        (margin, margin)
    ]
    goal_point = path_planner.PATHS[0][-1]

    waypoint_index = 0
    print(f"[PATH] Created {len(path_planner.PATHS[0])} waypoints to trace corners.")
    print(f"[PATH] Start={leader}, End={goal_point}")

    while True:
        cur_locs, cur_headings = vision.locate_robots()

        if len(cur_locs) < 1 or cur_locs[0] == (0, 0):
            print("Leader tag lost. Not sending command.")
            time.sleep(0.2)
            continue

        cur_loc = cur_locs[0]
        cur_heading = cur_headings[0]

        x, y = cur_loc

        # Safety stop if outside arena
        if x < -10 or x > 92 or y < -10 or y > 92:
            print(f"[SAFETY] Robot out of arena at {cur_loc}. Sending stop.")
            next_loc = cur_loc

        else:
            path = path_planner.PATHS[0]

            # Final stop buffer
            final_dist = dist(cur_loc, goal_point)

            if final_dist <= FINAL_RADIUS_CM and waypoint_index >= len(path) - 1:
                print(f"[GOAL] Within {FINAL_RADIUS_CM}cm of final target. Stopping.")
                next_loc = cur_loc

            else:
                # Advance waypoint while robot is close enough
                while waypoint_index < len(path) - 1 and dist(cur_loc, path[waypoint_index]) <= WAYPOINT_RADIUS_CM:
                    waypoint_index += 1

                next_loc = path[waypoint_index]

        sent = network.send(0, data=[[cur_heading, 0], cur_loc, next_loc])

        print(
            f"[SEND] Robot 0 sent={sent} "
            f"cur={cur_loc} heading={cur_heading} "
            f"wp_index={waypoint_index}/{len(path_planner.PATHS[0]) - 1} "
            f"next={next_loc} final={goal_point} "
            f"dist_to_next={dist(cur_loc, next_loc):.1f}cm "
            f"dist_to_final={dist(cur_loc, goal_point):.1f}cm"
        )

        time.sleep(0.20)

if __name__ == "__main__":
    main()