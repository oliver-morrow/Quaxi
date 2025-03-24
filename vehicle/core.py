# Import necessary modules
from robot_hat import Pin, ADC, PWM, Servo, fileDB
from robot_hat import Grayscale_Module, Ultrasonic, utils
import time
import os

# Helper function
def limit_value(value, minimum, maximum):
    '''
    Constrains a value within a specified range.
    '''
    return max(minimum, min(maximum, value))

class RobotVehicle(object):
    """Primary vehicle hardware interface class"""
    CONFIG_PATH = '/opt/picar-x/picar-x.conf'

    # Default sensor calibration values
    DEFAULT_LINE_SENSING = [1000, 1000, 1000]
    DEFAULT_CLIFF_SENSING = [500, 500, 500]

    # Movement constraints
    STEERING_MIN = -30
    STEERING_MAX = 30
    CAMERA_PAN_MIN = -90
    CAMERA_PAN_MAX = 90
    CAMERA_TILT_MIN = -35
    CAMERA_TILT_MAX = 65

    # PWM settings
    PWM_PERIOD = 4095
    PWM_PRESCALER = 10
    TIMEOUT = 0.02

    def __init__(self, debug=None):
        """Initialize the robot vehicle hardware"""
        self.debug = debug
        
        if self.debug:
            self.debug.info("Initializing vehicle hardware")
            
        # Load configuration file
        self.config = fileDB(self.CONFIG_PATH, 774, os.getlogin())
        
        # Motor initialization
        self.left_rear_dir_pin = Pin(self.config.get('left_rear_dir_pin', 13))
        self.right_rear_dir_pin = Pin(self.config.get('right_rear_dir_pin', 12))
        self.left_rear_pwm_pin = PWM(self.config.get('left_rear_pwm_pin', 8))
        self.right_rear_pwm_pin = PWM(self.config.get('right_rear_pwm_pin', 9))
        
        # Servo initialization for steering and camera
        self.steering_servo_pin = Servo(self.config.get('steering_servo_pin', 2))
        self.camera_servo_pan_pin = Servo(self.config.get('camera_servo_pan_pin', 6))
        self.camera_servo_tilt_pin = Servo(self.config.get('camera_servo_tilt_pin', 7))
        
        # Sensor initialization
        self.grayscale_module = Grayscale_Module(
            self.config.get('grayscale_pins', (25, 24, 23)), 
            self.config.get('reference_default', 1000)
        )
        self.ultrasonic = Ultrasonic(
            self.config.get('ultrasonic_trig_pin', 27),
            self.config.get('ultrasonic_echo_pin', 22)
        )
        
        # Initialize PWM configuration
        self.left_rear_pwm_pin.period(self.PWM_PERIOD)
        self.right_rear_pwm_pin.period(self.PWM_PERIOD)
        self.left_rear_pwm_pin.prescaler(self.PWM_PRESCALER)
        self.right_rear_pwm_pin.prescaler(self.PWM_PRESCALER)
        
        # Set initial states
        self.set_steering_angle(0)
        self.set_camera_pan(0)
        self.set_camera_tilt(0)
        
        # Default power levels
        self.motor_power = int(self.config.get('motor_power', 50))
        
        if self.debug:
            self.debug.info("Vehicle hardware initialized successfully")
    
    def forward(self, power=None):
        """Drive forward with specified power level"""
        if power is None:
            power = self.motor_power
        
        self.left_rear_dir_pin.value(0)
        self.right_rear_dir_pin.value(1)
        self.left_rear_pwm_pin.pulse_width_percent(power)
        self.right_rear_pwm_pin.pulse_width_percent(power)
        
        if self.debug:
            self.debug.debug(f"Forward motion, power: {power}")
    
    def backward(self, power=None):
        """Drive backward with specified power level"""
        if power is None:
            power = self.motor_power
        
        self.left_rear_dir_pin.value(1)
        self.right_rear_dir_pin.value(0)
        self.left_rear_pwm_pin.pulse_width_percent(power)
        self.right_rear_pwm_pin.pulse_width_percent(power)
        
        if self.debug:
            self.debug.debug(f"Backward motion, power: {power}")
    
    def stop(self):
        """Stop all motors"""
        self.left_rear_pwm_pin.pulse_width_percent(0)
        self.right_rear_pwm_pin.pulse_width_percent(0)
        
        if self.debug:
            self.debug.debug("Motors stopped")
    
    def set_steering_angle(self, angle):
        """Set the steering servo angle"""
        angle = limit_value(angle, self.STEERING_MIN, self.STEERING_MAX)
        self.steering_servo_pin.angle(angle)
        
        if self.debug:
            self.debug.debug(f"Steering angle set to {angle}")
        
        return angle
    
    def set_camera_pan(self, angle):
        """Set the camera pan servo angle"""
        angle = limit_value(angle, self.CAMERA_PAN_MIN, self.CAMERA_PAN_MAX)
        self.camera_servo_pan_pin.angle(angle)
        
        if self.debug:
            self.debug.verbose(f"Camera pan angle set to {angle}")
        
        return angle
    
    def set_camera_tilt(self, angle):
        """Set the camera tilt servo angle"""
        angle = limit_value(angle, self.CAMERA_TILT_MIN, self.CAMERA_TILT_MAX)
        self.camera_servo_tilt_pin.angle(angle)
        
        if self.debug:
            self.debug.verbose(f"Camera tilt angle set to {angle}")
        
        return angle
    
    def get_grayscale_data(self):
        """Get raw grayscale sensor data"""
        return self.grayscale_module.get_grayscale_data()
    
    def get_line_status(self, gm_values=None):
        """Get line detection status from grayscale module"""
        if gm_values is None:
            gm_values = self.get_grayscale_data()
        return self.grayscale_module.get_line_status(gm_values)
    
    def get_distance(self):
        """Get distance measurement from ultrasonic sensor"""
        try:
            distance = self.ultrasonic.read()
            
            if self.debug:
                self.debug.verbose(f"Distance reading: {distance}cm")
                
            return distance
        except Exception as e:
            if self.debug:
                self.debug.warning(f"Error reading distance sensor: {e}")
            return None
    
    def reset_systems(self):
        """Reset all vehicle systems to default state"""
        self.stop_motors()
        self.set_steering_angle(0)
        self.set_camera_pan(0)
        self.set_camera_tilt(0)
        
        if self.debug:
            self.debug.info("Vehicle systems reset")
    
    def stop_motors(self):
        """Stop all motors safely"""
        self.stop()
