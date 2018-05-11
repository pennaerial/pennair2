from enum import Enum

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
        self.last_increment_dim = "y"

    def run(self):
        if self._reached_setpoint():
            self._update_velocity()
        else:
            #If altitude is too low, add some updward momentum
            #If too high, add some downward momentum
            pass

    def _reached_setpoint(self):
        if self.last_pose is None: 
            return True
        return False
        #TODO: Don't use distance_to_target, just see if last_pose.pose.position.x/y < currentpose.x/y
        return self.uav.distance_to_target(target=self.last_pose) < 2

    def _update_velocity(self):
        if self.last_pose is None:
            x = 0
            y = 0
            z = 0
        else: 
            x = self.last_pose.pose.position.x
            y = self.last_pose.pose.position.y
            z = self.height

        if self.last_increment_dim is "x":
            y += self.last_increment
            self.uav.set_velocity( (0, 1, 0) )
        else:
            self.last_increment += self.increment
            x += self.last_increment
            self.uav.set_velocity( (1, 0, 0) )

        self.last_pose = self.uav.generate_pose_stamped( (x, y, z) )

