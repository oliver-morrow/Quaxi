import cv2
import numpy as np
from picarx import Picarx
import time

class LaneDetector:
    def __init__(self, picarx_instance=None):
        self.px = picarx_instance if picarx_instance else Picarx()
        self.cap = cv2.VideoCapture(0)
        self.NEUTRAL_ANGLE = -13.5
        self.CAMERA_TILT_ANGLE = -20
        self.CAMERA_PAN_ANGLE = -10  # Further adjust to turn the camera more to the left
        self.running = False
        self.detection_thread = None
        
        # Set camera angle
        self.px.set_cam_tilt_angle(self.CAMERA_TILT_ANGLE)
        self.px.set_cam_pan_angle(self.CAMERA_PAN_ANGLE)

    def preprocess_image(self, frame):
        """Apply color filtering to isolate white and yellow lanes."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # White lane detection
        white_lower = np.array([0, 0, 180])
        white_upper = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        # Yellow lane detection with expanded hue range
        yellow_lower = np.array([15, 100, 100])
        yellow_upper = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
        return combined_mask

    def detect_lines(self, mask):
        """Detect lines using Hough Transform."""
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=150)
        return lines if lines is not None else []

    def calculate_lane_center(self, lines, frame_width):
        """Calculate lane center based on detected lines."""
        left_x, right_x = [], []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 1e-6)
            if -0.5 < slope < 0.5:  # Exclude horizontal lines
                continue
            (left_x if slope < 0 else right_x).append((x1 + x2) // 2)

        if left_x and right_x:
            lane_center = (np.mean(left_x) + np.mean(right_x)) // 2
        elif left_x:
            lane_center = np.mean(left_x)
        elif right_x:
            lane_center = np.mean(right_x)
        else:
            lane_center = frame_width // 2

        return int(lane_center)

    def lane_follow(self, show_visualization=False):
        """Process one frame for lane following, return steering angle."""
        ret, frame = self.cap.read()
        if not ret:
            return self.NEUTRAL_ANGLE

        # Crop the frame to only the lower 75% of the image
        height, width = frame.shape[:2]
        lower_75_percent_frame = frame[int(height * 0.25):, :]  # 75% of the lower part

        mask = self.preprocess_image(lower_75_percent_frame)
        lines = self.detect_lines(mask)
        lane_center = self.calculate_lane_center(lines, lower_75_percent_frame.shape[1])

        # Adjust steering
        steering_adjustment = np.clip((lane_center - lower_75_percent_frame.shape[1] // 2) * 0.03, -30, 30)
        final_angle = self.NEUTRAL_ANGLE + steering_adjustment

        # Draw visualization if needed
        if show_visualization and lines:
            # Draw lines on original frame for visualization
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Adjust line drawing to fit in the lower 75% of the frame
                y_offset = int(height * 0.25)  # Offset lines to fit in the lower 75%
                cv2.line(frame, (x1, y1 + y_offset), (x2, y2 + y_offset), (0, 255, 0), 3)

            # Draw the lane center marker on the original frame
            cv2.circle(frame, (lane_center, height // 2), 5, (0, 0, 255), -1)
            cv2.imshow("Lane Detection", frame)
            cv2.waitKey(1)

        return final_angle

    def start(self, speed=20):
        """Start lane following."""
        self.px.forward(speed)
        self.running = True
        while self.running:
            steering_angle = self.lane_follow(show_visualization=True)
            self.px.set_dir_servo_angle(steering_angle)
            time.sleep(0.05)  # Small delay to prevent overloading CPU

    def stop(self):
        """Stop lane following."""
        self.running = False
        self.px.stop()
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        detector = LaneDetector()
        print("Starting lane detection, press Ctrl+C to stop...")
        detector.start(speed=20)
    except KeyboardInterrupt:
        print("Stopping lane detection...")
    finally:
        detector.stop()
