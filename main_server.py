import numpy as np
import time

from pi_apriltag_tracker import Vision
from swarm_network import JetsonNetwork
import config
import path_planner


# =========================
# Tuning constants
# =========================

ARENA_MIN_CM = 0.0
ARENA_MAX_CM = 82.0

WAYPOINT_RADIUS_CM = 6.0
FINAL_RADIUS_CM = 8.0

SEND_PERIOD_SEC = 0.20


def dist(a, b):
    return float(
        np.linalg.norm(
            np.array(a, dtype=float) - np.array(b, dtype=float)
        )
    )


def inside_arena(pos):
    x, y = pos
    return (
        ARENA_MIN_CM <= x <= ARENA_MAX_CM
        and ARENA_MIN_CM <= y <= ARENA_MAX_CM
    )


def main():
    network = JetsonNetwork()
    vision = Vision(show_feed=True)

    network.start(config.NUM_DEVICES)

    print("[START] Waiting for leader tag ID 0...")
    print("[START] Need corner tags 10, 11, 12, 13 and robot tag 0 visible.")

    # =========================
    # Wait for leader
    # =========================
    while True:
        cur_locs, cur_headings = vision.locate_robots()
        network.poll_for_clients()

        if len(cur_locs) >= 1 and cur_locs[0] != (0, 0):
            leader = cur_locs[0]
            leader_heading = cur_headings[0]
            print(f"[VISION] Leader detected at {leader}, heading={leader_heading}")
            break

        print("[VISION] Waiting for leader tag ID 0...")
        time.sleep(0.2)

    # =========================
    # Create path
    # =========================
    path_planner.PATHS[0] = path_planner.create_one_path(
        np.array(leader),
        np.array(config.END_POINT)
    )

    path = path_planner.PATHS[0]
    waypoint_index = 0

    print(f"[PATH] Created {len(path)} points.")
    print(f"[PATH] Start={leader}, End={config.END_POINT}")
    print(f"[PATH] Points={path}")

    # =========================
    # Latching stop states
    # =========================
    goal_reached = False
    boundary_stop = False
    stop_reason = None

    # =========================
    # Main loop
    # =========================
    while True:
        cur_locs, cur_headings = vision.locate_robots()
        network.poll_for_clients()

        if len(cur_locs) < 1 or cur_locs[0] == (0, 0):
            print("[SAFETY] Leader tag lost. Not sending command.")
            time.sleep(0.2)
            continue

        cur_loc = cur_locs[0]
        cur_heading = cur_headings[0]

        final_dist = dist(cur_loc, config.END_POINT)

        # =========================
        # Hard boundary stop
        # =========================
        if not inside_arena(cur_loc):
            boundary_stop = True
            stop_reason = f"OUT_OF_BOUNDS at {cur_loc}"

        # =========================
        # Hard goal stop
        # =========================
        if final_dist <= FINAL_RADIUS_CM:
            goal_reached = True
            stop_reason = f"GOAL_REACHED dist={final_dist:.1f}cm"

        # =========================
        # If stopped, keep sending current location as goal forever
        # =========================
        if boundary_stop or goal_reached:
            next_loc = cur_loc

            sent = network.send(
                0,
                data=[[cur_heading, 0], cur_loc, next_loc]
            )

            print(
                f"[HARD_STOP] reason={stop_reason} | "
                f"sent={sent} | "
                f"cur={cur_loc} heading={cur_heading} | "
                f"next={next_loc} | final={config.END_POINT} | "
                f"dist_to_final={final_dist:.1f}cm"
            )

            time.sleep(SEND_PERIOD_SEC)
            continue

        # =========================
        # Waypoint advance logic
        # =========================
        path = path_planner.PATHS[0]

        while (
            waypoint_index < len(path) - 1
            and dist(cur_loc, path[waypoint_index]) <= WAYPOINT_RADIUS_CM
        ):
            waypoint_index += 1

        next_loc = path[waypoint_index]

        # Extra protection:
        # If next waypoint is somehow outside arena, stop forever.
        if not inside_arena(next_loc):
            boundary_stop = True
            stop_reason = f"NEXT_WAYPOINT_OUT_OF_BOUNDS {next_loc}"
            next_loc = cur_loc

        sent = network.send(
            0,
            data=[[cur_heading, 0], cur_loc, next_loc]
        )

        print(
            f"[SEND] Robot 0 sent={sent} | "
            f"cur={cur_loc} heading={cur_heading} | "
            f"wp_index={waypoint_index}/{len(path) - 1} | "
            f"next={next_loc} final={config.END_POINT} | "
            f"dist_to_next={dist(cur_loc, next_loc):.1f}cm | "
            f"dist_to_final={final_dist:.1f}cm"
        )

        time.sleep(SEND_PERIOD_SEC)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[STOP] main_server.py stopped by user.")