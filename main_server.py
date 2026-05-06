import numpy as np
import time

from pi_apriltag_tracker import Vision
from swarm_network import JetsonNetwork
import config


# =========================
# Tuning constants
# =========================

ARENA_MIN_CM = 0.0
ARENA_MAX_CM = 82.0

# Hard stop if robot gets too close to border
BOUNDARY_MARGIN_CM = 3.0

# Stop permanently when close enough to final goal
FINAL_RADIUS_CM = 8.0

# Stop/advance when close enough to current axis waypoint
WAYPOINT_RADIUS_CM = 6.0

SEND_PERIOD_SEC = 0.20


# =========================
# Helper functions
# =========================

def dist(a, b):
    return float(
        np.linalg.norm(
            np.array(a, dtype=float) - np.array(b, dtype=float)
        )
    )


def inside_safe_arena(pos):
    x, y = pos

    return (
        ARENA_MIN_CM + BOUNDARY_MARGIN_CM <= x <= ARENA_MAX_CM - BOUNDARY_MARGIN_CM
        and ARENA_MIN_CM + BOUNDARY_MARGIN_CM <= y <= ARENA_MAX_CM - BOUNDARY_MARGIN_CM
    )


def create_axis_path(start, end):
    """
    Creates a Manhattan / axis-aligned path.

    Robot moves:
    1. Along X direction first
    2. Then along Y direction

    Example:
    start = (13, 20)
    end   = (50, 40)

    path:
    (13, 20) -> (50, 20) -> (50, 40)
    """
    sx, sy = start
    ex, ey = end

    return [
        [int(sx), int(sy)],
        [int(ex), int(sy)],
        [int(ex), int(ey)]
    ]


def get_closest_path_index(path, cur_loc):
    path_arr = np.array(path, dtype=float)
    cur_arr = np.array(cur_loc, dtype=float)

    distances = np.linalg.norm(path_arr - cur_arr, axis=1)
    return int(np.argmin(distances))


# =========================
# Main
# =========================

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
    # Create axis-only path
    # =========================
    path = create_axis_path(
        leader,
        config.END_POINT
    )

    print(f"[PATH] Created axis-only path with {len(path)} points.")
    print(f"[PATH] Start={leader}, End={config.END_POINT}")
    print(f"[PATH] Points={path}")

    # Current waypoint index.
    # Start from 1 because path[0] is the robot's starting position.
    waypoint_index = 1

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
        if not inside_safe_arena(cur_loc):
            boundary_stop = True
            stop_reason = f"OUT_OF_BOUNDS_OR_TOO_CLOSE_TO_BOUNDARY at {cur_loc}"

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
        # Axis waypoint logic
        # =========================

        # If close to current waypoint, advance to next waypoint.
        if waypoint_index < len(path):
            current_waypoint = path[waypoint_index]

            if dist(cur_loc, current_waypoint) <= WAYPOINT_RADIUS_CM:
                print(
                    f"[WAYPOINT] Reached waypoint {waypoint_index}: "
                    f"{current_waypoint}. Advancing."
                )
                waypoint_index += 1

        # If all waypoints are done, stop.
        if waypoint_index >= len(path):
            goal_reached = True
            stop_reason = "ALL_WAYPOINTS_COMPLETE"
            next_loc = cur_loc
        else:
            next_loc = path[waypoint_index]

        # Extra protection: never send unsafe waypoint
        if not inside_safe_arena(next_loc):
            boundary_stop = True
            stop_reason = f"NEXT_WAYPOINT_UNSAFE {next_loc}"
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