class VehicleController:
    """High-level vehicle control functions"""
    
    def __init__(self, robot_vehicle):
        """Initialize with reference to the physical vehicle"""
        self.vehicle = robot_vehicle
        self.current_speed = 0
        self.current_steering = 0
        self.camera_angle = 0

    def drive_forward(self, speed=30):
        """Drive vehicle forward at specified speed"""
        self.current_speed = speed
        self.vehicle.forward(speed)

    def drive_backward(self, speed=30):
        """Drive vehicle backward at specified speed"""
        self.vehicle.backward(speed)
        self.current_speed = -speed

    def steer_left(self, speed=30, angle=-30):
        """Turn vehicle left while moving forward"""
        self.vehicle.set_dir_servo_angle(angle)
        self.current_steering = angle
        self.current_speed = speed
        self.vehicle.forward(speed)

    def steer_right(self, speed=30, angle=30):
        """Turn vehicle right while moving forward"""
        self.vehicle.set_dir_servo_angle(angle)
        self.current_steering = angle
        self.current_speed = speed
        self.vehicle.forward(speed)

    def halt(self):
        """Stop all vehicle movement"""
        self.vehicle.stop()
        self.current_speed = 0
        
    def set_camera_angle(self, angle):
        """Adjust camera pan angle for object tracking"""
        self.vehicle.set_cam_pan_angle(angle)
        self.camera_angle = angle
