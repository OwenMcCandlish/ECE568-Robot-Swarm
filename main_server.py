import numpy as np
import time

from pi_apriltag_tracker import Vision
from swarm_network import JetsonNetwork
import config


# =========================
# Arena / safety settings
# =========================

ARENA_MIN_CM = 0.0
ARENA_MAX_CM = 82.0

# Keep robot away from actual boundary tags
BOUNDARY_MARGIN_CM = 3.0

# Robot must get within this distance of a corner target to count as reached
CORNER_RADIUS_CM = 15.0

# Send rate to ESP32
SEND_PERIOD_SEC = 0.20

# If True, robot keeps going corner to corner forever.
# If False, robot stops after one full corner loop.
LOOP_FOREVER = False


# =========================
# Safe corner targets
# =========================
# Coordinate system:
# Tag 13 = (0, 0)      bottom-left
# Tag 12 = (82, 0)     bottom-right
# Tag 11 = (82, 82)    top-right
# Tag 10 = (0, 82)     top-left
#
# We use safe inside-corners:
# bottom-left  = (8, 8)
# bottom-right = (74, 8)
# top-right    = (74, 74)
# top-left     = (8, 74)

CORNER_TARGETS = [
    (8, 8),
    (74, 8),
    (74, 74),
    (8, 74),
]


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


def nearest_corner_index(pos):
    distances = [dist(pos, corner) for corner in CORNER_TARGETS]
    return int(np.argmin(distances))


# =========================
# Main
# =========================

def main():
    network = JetsonNetwork()
    vision = Vision(show_feed=True)

    network.start(config.NUM_DEVICES)

    print("[START] Corner-to-corner mode")
    print("[START] Need corner tags 10, 11, 12, 13 and robot tag 0 visible.")
    print(f"[CORNERS] {CORNER_TARGETS}")

    # =========================
    # Wait for leader tag 0
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
    # Choose first target
    # =========================
    # If robot starts near a corner, go to the next corner in order.
    # Example: near bottom-left -> target bottom-right.
    start_corner_index = nearest_corner_index(leader)
    target_index = (start_corner_index + 1) % len(CORNER_TARGETS)

    completed_targets = 0

    print(f"[START] Nearest corner index: {start_corner_index}, corner={CORNER_TARGETS[start_corner_index]}")
    print(f"[TARGET] First target index: {target_index}, target={CORNER_TARGETS[target_index]}")

    # =========================
    # Latching stop states
    # =========================
    hard_stop = False
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

        # =========================
        # Hard boundary stop
        # =========================
        if not inside_safe_arena(cur_loc):
            hard_stop = True
            stop_reason = f"OUT_OF_BOUNDS_OR_TOO_CLOSE_TO_BOUNDARY at {cur_loc}"

        # =========================
        # If hard stopped, keep sending current location forever
        # =========================
        if hard_stop:
            next_loc = cur_loc

            sent = network.send(
                0,
                data=[[cur_heading, 0], cur_loc, next_loc]
            )

            print(
                f"[HARD_STOP] reason={stop_reason} | "
                f"sent={sent} | "
                f"cur={cur_loc} heading={cur_heading} | "
                f"next={next_loc}"
            )

            time.sleep(SEND_PERIOD_SEC)
            continue

        # =========================
        # Current target corner
        # =========================
        target = CORNER_TARGETS[target_index]
        d_to_target = dist(cur_loc, target)

        # =========================
        # Reached current corner target
        # =========================
        if d_to_target <= CORNER_RADIUS_CM:
            print(
                f"[CORNER_REACHED] target_index={target_index} "
                f"target={target} dist={d_to_target:.1f}cm"
            )

            completed_targets += 1

            # Stop after one full loop if LOOP_FOREVER is False
            if not LOOP_FOREVER and completed_targets >= len(CORNER_TARGETS):
                hard_stop = True
                stop_reason = "COMPLETED_ONE_FULL_CORNER_LOOP"
                next_loc = cur_loc
            else:
                target_index = (target_index + 1) % len(CORNER_TARGETS)
                target = CORNER_TARGETS[target_index]
                next_loc = target

                print(f"[TARGET] New target_index={target_index}, target={target}")

        else:
            next_loc = target

        # =========================
        # Send target to ESP32
        # =========================
        sent = network.send(
            0,
            data=[[cur_heading, 0], cur_loc, next_loc]
        )

        print(
            f"[SEND] Robot 0 sent={sent} | "
            f"cur={cur_loc} heading={cur_heading} | "
            f"target_index={target_index} | "
            f"next={next_loc} | "
            f"dist_to_target={dist(cur_loc, next_loc):.1f}cm | "
            f"completed={completed_targets}/{len(CORNER_TARGETS)}"
        )

        time.sleep(SEND_PERIOD_SEC)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[STOP] main_server.py stopped by user.")