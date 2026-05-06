import math

# =========================
# Arena / robot constants
# =========================

# Your arena is defined by AprilTag corners 10, 11, 12, 13
ARENA_SIZE_CM = 82.0

BOT_LENGTH_CM = 20
BOT_WIDTH_CM = 10

FOLLOW_DIST_CM = 30

MAX_V = 10                    # max linear speed in cm/sec
MAX_W = math.pi / 4           # max angular speed in rad/sec

LOOKAHEAD_LEN = 15q           # number of path points to look ahead
INTERVAL = 0.1

MAX_E = 20 * MAX_V * INTERVAL

NET_TIMEOUT = 2

# Leader-only test
NUM_DEVICES = 1

# Goal inside 82 cm x 82 cm arena
# Try this first. You can change it later.
END_POINT = (50, 40)

# Motor scaling on ESP32
DC_SCALE = 3

# Network
PI_IP = "172.20.10.2"