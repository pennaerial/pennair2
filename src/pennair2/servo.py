
class Servo(Mavros):

    def __init__(self, servo_port, servo_min, servo_max, init_angle):

        self.servo_port = servo_port Mavros
        self.servo_min = servo_min
        self.servo_max = servo_max
        self.angle = init_angle

    def set_servo_angle(self, angle):
        self.angle = min(self.servo_max, max(self.servo_min, angle))

    def get_servo_angle(self):
        return self.angle

    def set_servo(self):
        return self.angle
