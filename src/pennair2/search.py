from enum import Enum
from PID import PID
from pennair2.conversions import to_pose_stamped


class FoundAction(Enum):
    CONTINUE = 1
    STOP = 2

class Search(object):
    def __init__(self, uav, height=10):
        self.found_action = FoundAction.CONTINUE
        self.uav = uav
        self.start_location = self.uav.get_pose()
        self.height = height

    def run(self):
        print("Cannot run basic Search instance, instantiate a specific search type instead")

    def is_target(self):
        return False

    def on_target_found(self):
        pass

class BoxSearch(Search):
    def __init__(self, uav, height=10, radius=None, increment=5):
        super(BoxSearch, self).__init__(uav, height=height)
        self.radius = radius
        self.increment = increment
        self.last_pose = None
        self.last_increment = self.increment // 2
        self.last_increment_dim = "-y"
        self.velocity = None
        self.pidz = PID(1.2, 1, .001)
        self.pidz.SetPoint = self.height
        self.pidz.setSampleTime(.01)
        self.pidxy = None

    def run(self):
        if self._reached_setpoint():
            self._update_velocity()
        else:
            self._pid_loop()

    def _pid_loop(self):
        if self.velocity is None:
            print("WARN: BoxSearch -> no velocity setpoint")
            return
        x, y, _ = self.velocity
        cx, cy, cz = self.uav.get_position()
        if x == 0:
            self.pidxy.update(cx)
            x = self.pidxy.output
        else: #y = 0
            self.pidxy.update(cy)
            y = self.pidxy.output
        self.pidz.update(cz)
        self.uav.set_velocity( (x, y, self.pidz.output) )

    def _reached_setpoint(self):
        if self.last_pose is None: 
            return True
        
        x, y, _ = self.uav.get_pose(fmt="tuple")
        if self.last_increment_dim == "+x":
            return x > self.last_pose.pose.position.x
        elif self.last_increment_dim == "-x":
            return x < self.last_pose.pose.position.x
        elif self.last_increment_dim == "+y":
            return y > self.last_pose.pose.position.y
        elif self.last_increment_dim == "-y":
            return y < self.last_pose.pose.position.y
        else:
            print("WARN: Invalid increment_dim")
            return False

    def _update_velocity(self):
        if self.last_pose is None:
            x = 0
            y = 0
            z = 0
        else: 
            x = self.last_pose.pose.position.x
            y = self.last_pose.pose.position.y
            z = self.height

        v = None
        self.pidxy = PID(1.2, 1, .001)
        self.pidxy.setSampleTime(.01)
        if self.last_increment_dim == "+x":     #+y
            y += self.last_increment
            self.velocity = (0, 1, 0)
            self.last_increment_dim = "+y"
            self.pidxy.SetPoint = x
        elif self.last_increment_dim == "+y":   #-x
            self.last_increment += self.increment
            x -= self.last_increment
            self.velocity = (-1, 0, 0)
            self.last_increment_dim = "-x"
            self.pidxy.SetPoint = y
        elif self.last_increment_dim == "-x":   #-y
            y -= self.last_increment
            self.velocity = (0, -1, 0)
            self.last_increment_dim = "-y"
            self.pidxy.SetPoint = x
        elif self.last_increment_dim == "-y":   #+x
            self.last_increment += self.increment
            x += self.last_increment
            self.velocity = (1, 0, 0)
            self.last_increment_dim = "+x"
            self.pidxy.SetPoint = y
        else:
            print("WARN: Invalid increment_dim (_update_velocity)")

        self._pid_loop()
        self.last_pose = to_pose_stamped((x, y, z))

