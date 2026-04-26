##########################################################################################################
# imports
import math

##########################################################################################################
# Parameter                # Value                                      # Explanation

PIXEL_LENGTH                = 2                                         # cm per pixel
BOT_LENGTH_CM               = 20                                        # bot length in cm
BOT_LENGTH_PIXEL            = BOT_LENGTH_CM // PIXEL_LENGTH             # bot length in pixels
BOT_WIDTH_CM                = 10                                        # bot length in cm
BOT_WIDTH_PIXEL             = BOT_WIDTH_CM // PIXEL_LENGTH              # bot length in pixels
FOLLOW_DIST_CM              = 10                                        # follow distance in cm
FOLLOW_DIST_PIXEL           = FOLLOW_DIST_CM // PIXEL_LENGTH            # follow distance in pixels

MAX_V                       = 10                                        # max cm/sec
MAX_W                       = math.pi / 4                               # max rad/sec

LOOKAHEAD_LEN               = 5                                         # waypoints to lookahead (& send)

INTERVAL                    = 0.1                                       # seconds per interval

MAX_E                       = 20 * MAX_V * INTERVAL                     # Distance in which we start slowing down


NUM_DEVICES = 3
END_POINT = (0, 0) # TODO


# Jetson Parameters
SSID: str = "jetson" # TODO
PWD: str = "12345"    # TODO
