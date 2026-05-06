from picamera2 import Picamera2
import cv2
import numpy as np
from pupil_apriltags import Detector
import math
import time
import csv
import os

try:
    import config
except ImportError:
    config = None


# =========================
# Vision Class for Integration
# =========================
class Vision:
    def __init__(self, show_feed=False):
        self.HEADING_OFFSET_DEG = 180
        self.TAG_FAMILY = "tag36h11"
        self.SMOOTHING_ALPHA = 0.2
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 480

        self.detector = Detector(
            families=self.TAG_FAMILY,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25
        )

        self.picam2 = Picamera2()
        cfg = self.picam2.create_preview_configuration(
            main={
                "size": (self.FRAME_WIDTH, self.FRAME_HEIGHT),
                "format": "RGB888"
            }
        )
        self.picam2.configure(cfg)
        self.picam2.start()
        time.sleep(1) # Warm up

        self.smoothed_states = {}

        self.ARENA_SIZE_CM = 82.0

        self.CORNER_TAG_IDS = {
            10: "top_left",
            11: "top_right",
            12: "bottom_right",
            13: "bottom_left",
        }

        self.corner_centers = {}
        self.H_IMAGE_TO_WORLD = None

        self.show_feed = show_feed
        self.ARROW_LENGTH = 50

    def _smooth_angle_deg(self, old_deg, new_deg, alpha):
        old_rad = math.radians(old_deg)
        new_rad = math.radians(new_deg)

        old_x, old_y = math.cos(old_rad), math.sin(old_rad)
        new_x, new_y = math.cos(new_rad), math.sin(new_rad)

        blended_x = (1 - alpha) * old_x + alpha * new_x
        blended_y = (1 - alpha) * old_y + alpha * new_y

        return math.degrees(math.atan2(blended_y, blended_x))

    def _update_arena_homography(self, results):
        """
        Uses corner AprilTags 10, 11, 12, 13 to build a pixel-to-cm transform.
        Arena is 82 cm x 82 cm.
        """
        for r in results:
            if r.tag_id in self.CORNER_TAG_IDS:
                self.corner_centers[r.tag_id] = np.array(r.center, dtype=np.float32)

        required = [10, 11, 12, 13]
        if not all(tag_id in self.corner_centers for tag_id in required):
            self.H_IMAGE_TO_WORLD = None
            return False

        image_points = np.array([
            self.corner_centers[10],  # top-left
            self.corner_centers[11],  # top-right
            self.corner_centers[12],  # bottom-right
            self.corner_centers[13],  # bottom-left
        ], dtype=np.float32)

        world_points = np.array([
            [0.0, self.ARENA_SIZE_CM],               # 10: top-left -> x=0, y=82
            [self.ARENA_SIZE_CM, self.ARENA_SIZE_CM],# 11: top-right -> x=82, y=82
            [self.ARENA_SIZE_CM, 0.0],               # 12: bottom-right -> x=82, y=0
            [0.0, 0.0],                              # 13: bottom-left -> x=0, y=0
        ], dtype=np.float32)

        self.H_IMAGE_TO_WORLD, _ = cv2.findHomography(image_points, world_points)
        return self.H_IMAGE_TO_WORLD is not None

    def _pixel_to_world_cm(self, x_px, y_px):
        """
        Converts image pixel coordinate to real arena cm coordinate.
        """
        if self.H_IMAGE_TO_WORLD is None:
            return None

        point = np.array([[[x_px, y_px]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(point, self.H_IMAGE_TO_WORLD)

        x_cm = float(transformed[0][0][0])
        y_cm = float(transformed[0][0][1])

        return x_cm, y_cm

    def locate_robots(self) -> tuple[list[tuple[int, int]], list[int]]:
        frame_rgb = self.picam2.capture_array()
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)

        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        results = self.detector.detect(gray)
        robots_raw = {}

        arena_ready = self._update_arena_homography(results)

        if not arena_ready:
            print("Arena corner tags not fully visible. Need IDs 10, 11, 12, 13.")

        for r in results:
            tag_id = r.tag_id

            center = r.center
            corners = r.corners

            corners_int = corners.astype(int)
            center_int = tuple(center.astype(int))

            # Draw tag outline
            for j in range(4):
                pt1 = tuple(corners_int[j])
                pt2 = tuple(corners_int[(j + 1) % 4])
                cv2.line(frame, pt1, pt2, (255, 0, 0), 2)

            # Draw tag center
            cv2.circle(frame, center_int, 6, (0, 0, 255), -1)

            if tag_id in self.CORNER_TAG_IDS:
                cv2.putText(
                    frame,
                    f"Corner {tag_id}",
                    (center_int[0] + 10, center_int[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2
                )
                continue


            # Orientation from bottom-center to top-center
            bottom_center_x = (corners[0][0] + corners[1][0]) / 2.0
            bottom_center_y = (corners[0][1] + corners[1][1]) / 2.0
            top_center_x = (corners[3][0] + corners[2][0]) / 2.0
            top_center_y = (corners[3][1] + corners[2][1]) / 2.0

            if arena_ready:
                bottom_world = self._pixel_to_world_cm(bottom_center_x, bottom_center_y)
                top_world = self._pixel_to_world_cm(top_center_x, top_center_y)

                if bottom_world is None or top_world is None:
                    continue

                dx = top_world[0] - bottom_world[0]
                dy = top_world[1] - bottom_world[1]
            else:
                dx = top_center_x - bottom_center_x
                dy = top_center_y - bottom_center_y

            angle_rad = math.atan2(dy, dx)
            angle_deg = math.degrees(angle_rad) + self.HEADING_OFFSET_DEG

            while angle_deg > 180:
                angle_deg -= 360

            while angle_deg < -180:
                angle_deg += 360

            if arena_ready:
                world_pos = self._pixel_to_world_cm(float(center[0]), float(center[1]))

                if world_pos is None:
                    continue

                x_world, y_world = world_pos
            else:
                # Fallback: still use pixels if calibration is not ready
                x_world, y_world = float(center[0]), float(center[1])

            robots_raw[tag_id] = {
                "x_px": x_world,
                "y_px": y_world,
                "heading_deg": float(angle_deg)
            }

           # Draw heading arrow correctly by projecting world heading back into image pixels
            if arena_ready and self.H_IMAGE_TO_WORLD is not None:
                H_WORLD_TO_IMAGE = np.linalg.inv(self.H_IMAGE_TO_WORLD)

                arrow_len_cm = 12.0
                sa_rad = math.radians(angle_deg)

                start_world = np.array([[[x_world, y_world]]], dtype=np.float32)
                end_world = np.array([[
                    [
                        x_world + arrow_len_cm * math.cos(sa_rad),
                        y_world + arrow_len_cm * math.sin(sa_rad)
                    ]
                ]], dtype=np.float32)

                start_img = cv2.perspectiveTransform(start_world, H_WORLD_TO_IMAGE)
                end_img = cv2.perspectiveTransform(end_world, H_WORLD_TO_IMAGE)

                start_pt = (
                    int(start_img[0][0][0]),
                    int(start_img[0][0][1])
                )
                end_pt = (
                    int(end_img[0][0][0]),
                    int(end_img[0][0][1])
                )
            else:
                sa_rad = math.radians(angle_deg)
                start_pt = center_int
                end_pt = (
                    int(center_int[0] + self.ARROW_LENGTH * math.cos(sa_rad)),
                    int(center_int[1] + self.ARROW_LENGTH * math.sin(sa_rad))
                )

            cv2.arrowedLine(
                    frame,
                    start_pt,
                    end_pt,
                    (0, 255, 255),
                    2,
                    tipLength=0.25
            )

            cv2.putText(
                frame,
                f"ID:{tag_id}",
                (center_int[0] + 10, center_int[1] - 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2
            )

            cv2.putText(
                frame,
                f"X:{x_world:.1f} Y:{y_world:.1f} cm",
                (center_int[0] + 10, center_int[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2
            )

            cv2.putText(
                frame,
                f"A:{angle_deg:.1f} deg",
                (center_int[0] + 10, center_int[1] + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2
            )

            if tag_id not in self.smoothed_states:
                self.smoothed_states[tag_id] = robots_raw[tag_id].copy()
            else:
                self.smoothed_states[tag_id]["x_px"] = (
                    (1 - self.SMOOTHING_ALPHA) * self.smoothed_states[tag_id]["x_px"]
                    + self.SMOOTHING_ALPHA * robots_raw[tag_id]["x_px"]
                )
                self.smoothed_states[tag_id]["y_px"] = (
                    (1 - self.SMOOTHING_ALPHA) * self.smoothed_states[tag_id]["y_px"]
                    + self.SMOOTHING_ALPHA * robots_raw[tag_id]["y_px"]
                )
                self.smoothed_states[tag_id]["heading_deg"] = self._smooth_angle_deg(
                    self.smoothed_states[tag_id]["heading_deg"],
                    robots_raw[tag_id]["heading_deg"],
                    self.SMOOTHING_ALPHA
                )

        cur_locs = []
        cur_headings = []

        # We need config.NUM_DEVICES robots
        num_devices = config.NUM_DEVICES if config else 3

        for i in range(num_devices):
            if i in self.smoothed_states:
                x = int(self.smoothed_states[i]["x_px"])
                y = int(self.smoothed_states[i]["y_px"])
                h = int(self.smoothed_states[i]["heading_deg"])

                # eliminate negative headings
                if (h < 0):
                    h = 360 + h

                cur_locs.append((x, y))
                cur_headings.append(h)
            else:
                # Fallback if the robot hasn't been seen yet
                cur_locs.append((0, 0))
                cur_headings.append(0)
        if self.show_feed:
            cv2.imshow("Swarm Main Server Camera Feed", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.close()
                cv2.destroyAllWindows()
                raise KeyboardInterrupt

        return cur_locs, cur_headings

    def close(self):
        self.picam2.stop()


# =========================
# Standalone Debug Tracker
# =========================
def run_standalone():
    vision = Vision(show_feed=True)

    try:
        while True:
            cur_locs, cur_headings = vision.locate_robots()

            if cur_locs:
                print("Robots:", cur_locs, "Headings:", cur_headings)

            time.sleep(0.05)

    finally:
        vision.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    run_standalone()

