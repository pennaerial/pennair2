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
        nextPose = self._get_next_pose()
        self.uav.set_position(nextPose, blocking=True)
        #self.uav.wait(5)

    def _get_next_pose(self):
        if self.last_pose is None:
            self.last_pose = self.start_location
            return self.last_pose
        x = self.last_pose.pose.position.x
        y = self.last_pose.pose.position.y
        z = self.height

        if self.last_increment_dim is "x":
            y += self.last_increment
        if self.last_increment_dim is "y":
            self.last_increment = self.last_increment + self.increment
            x += self.last_increment
            self.last_increment *= -1

        heading = self.uav.get_heading()
        print("New pose: " + str( (x, y, z) ))
        
        return self.uav.generate_pose_stamped( (x, y, z), heading=heading)
