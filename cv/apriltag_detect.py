import cv2
import numpy as np
from pupil_apriltags import Detector
import math
import time
import csv
import os

# =========================
# Configuration
# =========================
CAMERA_INDEX = 0
TAG_FAMILY = "tag36h11"
SMOOTHING_ALPHA = 0.2   # higher = follows new values faster
ARROW_LENGTH = 50
CSV_FILE = "apriltag_log.csv"

# =========================
# AprilTag detector
# =========================
detector = Detector(
    families=TAG_FAMILY,
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25
)

# =========================
# Video capture
# =========================
cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    raise RuntimeError("Could not open webcam.")

# =========================
# CSV setup
# =========================
csv_exists = os.path.exists(CSV_FILE)
csv_file = open(CSV_FILE, mode="a", newline="")
csv_writer = csv.writer(csv_file)

if not csv_exists:
    csv_writer.writerow(["timestamp", "tag_id", "x_px", "y_px", "heading_deg"])

# =========================
# State for smoothing
# =========================
smoothed_states = {}

def smooth_angle_deg(old_deg, new_deg, alpha):
    """
    Smooth angle properly around wraparound boundaries like 179 -> -179.
    """
    old_rad = math.radians(old_deg)
    new_rad = math.radians(new_deg)

    old_x, old_y = math.cos(old_rad), math.sin(old_rad)
    new_x, new_y = math.cos(new_rad), math.sin(new_rad)

    blended_x = (1 - alpha) * old_x + alpha * new_x
    blended_y = (1 - alpha) * old_y + alpha * new_y

    return math.degrees(math.atan2(blended_y, blended_x))

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray)

        robots_raw = {}
        robots_smoothed = {}

        timestamp = time.time()

        for r in results:
            corners = r.corners
            center = r.center
            tag_id = r.tag_id

            # Convert corners and center to ints for drawing
            corners_int = corners.astype(int)
            center_int = tuple(center.astype(int))

            # Draw the box
            for i in range(4):
                pt1 = tuple(corners_int[i])
                pt2 = tuple(corners_int[(i + 1) % 4])
                cv2.line(frame, pt1, pt2, (255, 0, 0), 2)

            # Draw corner points
            for pt in corners_int:
                cv2.circle(frame, tuple(pt), 5, (0, 255, 0), -1)

            # Draw center
            cv2.circle(frame, center_int, 6, (0, 0, 255), -1)

            # Orientation from corner 0 -> corner 1
            dx = corners[1][0] - corners[0][0]
            dy = corners[1][1] - corners[0][1]
            angle_rad = math.atan2(dy, dx)
            angle_deg = math.degrees(angle_rad)

            # Raw state
            robots_raw[tag_id] = {
                "x_px": float(center[0]),
                "y_px": float(center[1]),
                "heading_deg": float(angle_deg)
            }

            # Apply smoothing
            if tag_id not in smoothed_states:
                smoothed_states[tag_id] = robots_raw[tag_id].copy()
            else:
                smoothed_states[tag_id]["x_px"] = (
                    (1 - SMOOTHING_ALPHA) * smoothed_states[tag_id]["x_px"]
                    + SMOOTHING_ALPHA * robots_raw[tag_id]["x_px"]
                )
                smoothed_states[tag_id]["y_px"] = (
                    (1 - SMOOTHING_ALPHA) * smoothed_states[tag_id]["y_px"]
                    + SMOOTHING_ALPHA * robots_raw[tag_id]["y_px"]
                )
                smoothed_states[tag_id]["heading_deg"] = smooth_angle_deg(
                    smoothed_states[tag_id]["heading_deg"],
                    robots_raw[tag_id]["heading_deg"],
                    SMOOTHING_ALPHA
                )

            robots_smoothed[tag_id] = smoothed_states[tag_id].copy()

            # Smoothed values for display
            sx = robots_smoothed[tag_id]["x_px"]
            sy = robots_smoothed[tag_id]["y_px"]
            sa = robots_smoothed[tag_id]["heading_deg"]
            sa_rad = math.radians(sa)

            # Draw heading arrow
            end_x = int(sx + ARROW_LENGTH * math.cos(sa_rad))
            end_y = int(sy + ARROW_LENGTH * math.sin(sa_rad))
            cv2.arrowedLine(
                frame,
                (int(sx), int(sy)),
                (end_x, end_y),
                (0, 255, 255),
                2,
                tipLength=0.25
            )

            # Draw text
            text1 = f"ID:{tag_id}"
            text2 = f"X:{sx:.1f} Y:{sy:.1f}"
            text3 = f"A:{sa:.1f} deg"

            cv2.putText(frame, text1, (center_int[0] + 10, center_int[1] - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            cv2.putText(frame, text2, (center_int[0] + 10, center_int[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            cv2.putText(frame, text3, (center_int[0] + 10, center_int[1] + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

            # Log to CSV
            csv_writer.writerow([
                timestamp,
                tag_id,
                robots_smoothed[tag_id]["x_px"],
                robots_smoothed[tag_id]["y_px"],
                robots_smoothed[tag_id]["heading_deg"]
            ])

        csv_file.flush()

        # Print per-frame robot states
        if robots_smoothed:
            print("Robots:", robots_smoothed)

        cv2.imshow("AprilTag Robot Tracker", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

finally:
    cap.release()
    csv_file.close()
    cv2.destroyAllWindows()

# import cv2
# import numpy as np
# from pupil_apriltags import Detector
# import math

# cap = cv2.VideoCapture(0)

# detector = Detector(
#     families="tag36h11",
#     nthreads=1,
#     quad_decimate=1.0,
#     quad_sigma=0.0,
#     refine_edges=1,
#     decode_sharpening=0.25
# )

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break

#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     results = detector.detect(gray)

#     for r in results:
#         corners = r.corners.astype(int)   # 4 corners
#         center = tuple(r.center.astype(int))
#         tag_id = r.tag_id

#         # Draw corners
#         for pt in corners:
#             cv2.circle(frame, tuple(pt), 5, (0, 255, 0), -1)

#         # Draw box
#         for i in range(4):
#             pt1 = tuple(corners[i])
#             pt2 = tuple(corners[(i + 1) % 4])
#             cv2.line(frame, pt1, pt2, (255, 0, 0), 2)

#         # Draw center
#         cv2.circle(frame, center, 6, (0, 0, 255), -1)

#         # Orientation:
#         # Use vector from corner 0 to corner 1
#         dx = corners[1][0] - corners[0][0]
#         dy = corners[1][1] - corners[0][1]
#         angle_rad = math.atan2(dy, dx)
#         angle_deg = math.degrees(angle_rad)

#         text = f"ID:{tag_id} C:{center} A:{angle_deg:.1f}"
#         cv2.putText(frame, text, (center[0] + 10, center[1] - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

#         print(f"Tag {tag_id}: center={center}, angle={angle_deg:.2f}")

#     cv2.imshow("AprilTag Detection", frame)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()
