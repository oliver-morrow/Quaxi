import cv2
import numpy as np

class EnhancedLineFollower:
    def __init__(self):
        self.last_steering_angle = 0
        self.steering_smoothing = 0.5  # Smoothing factor (0-1)
        
    def preprocess_image(self, image):
        """Convert image to grayscale and apply Gaussian blur"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        return blur
        
    def detect_edges(self, image):
        """Apply Canny edge detection"""
        # Apply threshold and Canny edge detection
        edges = cv2.Canny(image, 50, 150)
        return edges
        
    def region_of_interest(self, image):
        """Mask the image to only process the area of interest"""
        height, width = image.shape
        
        # Define a polygon for the mask (bottom half of the image)
        polygons = np.array([
            [(0, height), (width, height), (width, height//2), (0, height//2)]
        ])
        
        # Create a black mask
        mask = np.zeros_like(image)
        
        # Fill the mask with the polygon
        cv2.fillPoly(mask, polygons, 255)
        
        # Apply the mask to the image
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image
        
    def detect_line_segments(self, image):
        """Detect line segments using Hough transform"""
        # Define parameters for Hough transform
        rho = 1              # Distance resolution in pixels
        theta = np.pi/180    # Angular resolution in radians
        threshold = 10       # Minimum number of votes
        min_line_length = 8  # Minimum line length
        max_line_gap = 4     # Maximum allowed gap between points
        
        # Apply Hough transform
        line_segments = cv2.HoughLinesP(image, rho, theta, threshold, 
                                        np.array([]), min_line_length, max_line_gap)
        return line_segments
        
    def average_slope_intercept(self, image, line_segments):
        """Calculate average line segments by slope and position"""
        left_fit = []
        right_fit = []
        
        if line_segments is None:
            return None
            
        # Define area for considering left and right lane lines
        height, width = image.shape
        left_region_boundary = width * (1/3)  # Left lane line segment should be on left 1/3 of the screen
        right_region_boundary = width * (2/3) # Right lane line segment should be on right 2/3 of the screen
        
        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:  # Skip vertical lines
                    continue
                    
                # Calculate slope and intercept
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                
                # Group segments by slope and position
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))
        
        # Calculate average fits
        left_fit_average = np.average(left_fit, axis=0) if len(left_fit) > 0 else None
        right_fit_average = np.average(right_fit, axis=0) if len(right_fit) > 0 else None
        
        return left_fit_average, right_fit_average
        
    def calculate_steering_angle(self, image, lane_lines):
        """Calculate steering angle based on lane lines"""
        height, width = image.shape
        
        if lane_lines is None or len(lane_lines) == 0:
            # No lane lines detected, maintain previous steering
            return self.last_steering_angle
            
        # We need to determine center line based on lane lines
        # If there's only one lane line detected, use that
        if len(lane_lines) == 1:
            x1, y1, x2, y2 = lane_lines[0]
            x_offset = x2 - x1
        else:
            # Calculate intercepts at the bottom of the frame for both lines
            left_x1, left_y1, left_x2, left_y2 = lane_lines[0]
            right_x1, right_y1, right_x2, right_y2 = lane_lines[1]
            
            # Find midpoint at the bottom of the frame
            mid_x = width // 2
            left_x_at_bottom = left_x1 + (left_x2 - left_x1) * (height - left_y1) / (left_y2 - left_y1)
            right_x_at_bottom = right_x1 + (right_x2 - right_x1) * (height - right_y1) / (right_y2 - right_y1)
            
            # Calculate car's midpoint
            car_x = (left_x_at_bottom + right_x_at_bottom) // 2
            
            # Calculate offset from center
            x_offset = car_x - mid_x
        
        # Convert offset to angle: arctan(x/y)
        y_offset = height // 2
        angle_to_mid_radian = np.arctan(x_offset / y_offset)
        angle_to_mid_deg = np.rad2deg(angle_to_mid_radian)
        
        # Smooth the steering angle
        steering_angle = self.last_steering_angle * self.steering_smoothing + angle_to_mid_deg * (1 - self.steering_smoothing)
        self.last_steering_angle = steering_angle
        
        return steering_angle
    
    def process_frame(self, frame):
        """Process a frame to detect lane lines and calculate steering angle"""
        # Preprocess image
        preprocessed = self.preprocess_image(frame)
        
        # Detect edges
        edges = self.detect_edges(preprocessed)
        
        # Apply region of interest mask
        roi = self.region_of_interest(edges)
        
        # Detect line segments
        line_segments = self.detect_line_segments(roi)
        
        # Average segments by slope and position
        if line_segments is not None:
            left_fit, right_fit = self.average_slope_intercept(preprocessed, line_segments)
            
            # Create full lines from the fits
            lane_lines = []
            
            if left_fit is not None:
                height, width = preprocessed.shape
                left_line = self.make_points(height, width, left_fit)
                lane_lines.append(left_line)
                
            if right_fit is not None:
                height, width = preprocessed.shape
                right_line = self.make_points(height, width, right_fit)
                lane_lines.append(right_line)
                
            # Calculate steering angle
            if lane_lines:
                steering_angle = self.calculate_steering_angle(preprocessed, lane_lines)
                return steering_angle, lane_lines
        
        # If no lines detected, maintain previous steering
        return self.last_steering_angle, None
        
    def make_points(self, height, width, line):
        """Convert a line fit to line points"""
        slope, intercept = line
        
        # Make sure we have valid slope
        if slope == 0:
            slope = 0.1
            
        # Calculate coordinates
        y1 = height  # Bottom of the image
        y2 = int(height * 0.6)  # Slightly above the middle
        
        # Calculate x values
        x1 = max(0, min(width, int((y1 - intercept) / slope)))
        x2 = max(0, min(width, int((y2 - intercept) / slope)))
        
        return [x1, y1, x2, y2]
    
    def follow_line(self, line_data):
        """
        Process grayscale sensor data to determine steering angle
        
        Args:
            line_data: List of 3 boolean values from grayscale sensors
                       [left_sensor, center_sensor, right_sensor]
        
        Returns:
            Steering angle (-35 to 35 degrees)
        """
        # Default angle (go straight)
        angle = 0
        
        # Line following logic based on sensor data
        if line_data[0] and not line_data[2]:  # Line on left
            angle = -25  # Turn left
        elif line_data[2] and not line_data[0]:  # Line on right
            angle = 25   # Turn right
        elif line_data[0] and line_data[1] and line_data[2]:  # All sensors on
            angle = self.last_steering_angle  # Maintain current direction
        elif not line_data[0] and not line_data[1] and not line_data[2]:  # All sensors off
            angle = self.last_steering_angle  # Maintain current direction
        elif line_data[1]:  # Center sensor on
            angle = 0  # Go straight
            
        # Apply smoothing
        smooth_angle = self.last_steering_angle * self.steering_smoothing + angle * (1 - self.steering_smoothing)
        self.last_steering_angle = smooth_angle
        
        # Constrain angle to valid range
        return max(-35, min(35, smooth_angle))
    
    def get_steering_angle(self, frame):
        """Process frame and return steering angle"""
        steering_angle, _ = self.process_frame(frame)
        # Constrain steering angle to reasonable values
        return max(-35, min(35, steering_angle))
