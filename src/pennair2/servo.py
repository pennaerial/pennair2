<<<<<<< HEAD
from autopilot import Mavros


class Servo(Mavros):

    def __init__(self, servo_port, servo_min, servo_max):
=======

class Servo(Mavros):

    def __init__(self, servo_port, servo_min, servo_max, init_angle):
>>>>>>> e8a84e3acc1fa908ae56036b640698e1640cada0
        super(Servo, self).__init__()
        self.servo_port = servo_port
        self.servo_min = servo_min
        self.servo_max = servo_max
<<<<<<< HEAD
        self.angle = 0
        self.pwm = 1000
=======
        self.angle = init_angle
        self.pwm = 1000
        self.set_pwm()
>>>>>>> e8a84e3acc1fa908ae56036b640698e1640cada0

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
<<<<<<< HEAD
        super(Servo, self).send_command_long(183, [self.servo_port, self.pwm])
=======
        super(Servo, self).send_command_long(183, [183, self.pwm])
>>>>>>> e8a84e3acc1fa908ae56036b640698e1640cada0
