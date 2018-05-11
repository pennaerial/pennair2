from enum import Enum
from PID import PID

class FoundAction(Enum):
    CONTINUE = 1
    STOP = 2

class Search(object):
    def __init__(self, uav, height=10):
        self.found_action = FoundAction.CONTINUE
        self.uav = uav
        self.start_location = self.uav.get_position()
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
        self.pid = PID(1.2, 1, .001)
        self.pid.SetPoint = self.height
        self.pid.setSampleTime(.01)

    def run(self):
        if self._reached_setpoint():
            self._update_velocity()
        else:
            self._maintain_height()

    def _maintain_height(self):
        if self.velocity is None:
            print("WARN: BoxSearch -> no velocity setpoint")
            return
        x, y, _ = self.velocity
        _, _, z = self.uav.get_position(fmt="tuple")
        self.pid.update(z)
        self.uav.set_velocity( (x, y, self.pid.output) )

    def _reached_setpoint(self):
        if self.last_pose is None: 
            return True
        
        x, y, _ = self.uav.get_position(fmt="tuple")
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
        if self.last_increment_dim == "+x":     #+y
            y += self.last_increment
            self.velocity = (0, 1, 0)
            self.last_increment_dim = "+y"
        elif self.last_increment_dim == "+y":   #-x
            self.last_increment += self.increment
            x -= self.last_increment
            self.velocity = (-1, 0, 0)
            self.last_increment_dim = "-x"
        elif self.last_increment_dim == "-x":   #-y
            y -= self.last_increment
            self.velocity = (0, -1, 0)
            self.last_increment_dim = "-y"
        elif self.last_increment_dim == "-y":   #+x
            self.last_increment += self.increment
            x += self.last_increment
            self.velocity = (1, 0, 0)
            self.last_increment_dim = "+x"
        else:
            print("WARN: Invalid increment_dim (_update_velocity)")

        self._maintain_height()
        self.last_pose = self.uav.generate_pose_stamped( (x, y, z) )

