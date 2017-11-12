from autopilot import Mavros


class Servo(Mavros):

    def __init__(self, servo_port, servo_min, servo_max, init_angle):
        super(Servo, self).__init__()
        self.servo_port = servo_port
        self.servo_min = servo_min
        self.servo_max = servo_max
        self.angle = 0
        self.pwm = 1000
        self.angle = init_angle
        self.pwm = 1000
        self.set_pwm()

    def set_servo_angle(self, angle):
        self.angle = min(self.servo_max, max(self.servo_min, angle))
        self.set_pwm()
        self.set_servo()

    def get_servo_angle(self):
        return self.angle

    def __set_pwm(self):
        freq = 318.31*self.angle + 1000
        self.pwm = min(2000, max(1000, freq))

    def __set_servo(self):
        super(Servo, self).send_command_long(183, [self.servo_port, self.pwm])