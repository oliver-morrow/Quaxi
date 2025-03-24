class CarController:
    def __init__(self,car):
        self.car = car
        self.direction=0
        self.speed=0

    def move_forward(self, speed=30):
        self.speed=speed
        self.car.forward(speed)

    def move_backward(self, speed=30):
        self.car.backward(speed)
        self.speed=-speed

    def turn_left(self, speed=30, angle=-30):
        self.car.set_dir_servo_angle(angle)
        self.direction=angle
        self.speed=speed
        self.car.forward(speed)

    def turn_right(self, speed=30, angle=30):
        self.car.set_dir_servo_angle(angle)
        self.direction=angle
        self.speed=speed
        self.car.forward(speed)

    def stop(self):
        self.car.stop()
        self.speed=0
    def turnCam(self,angle):
        self.car.set_cam_pan_angle(angle)
        self.camAngle=angle

