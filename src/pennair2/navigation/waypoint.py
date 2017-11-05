#Simple class to manage waypoints
class Waypoints():

    #Define two fields, empty list of waypoints and index
    def __init__(self, frame):
        """
        :param frame: coordinate system
        :type frame: str
        """
        self.waypoints = []
        self.index = 0

    #add a waypoint by default to the end or at a specified index
    def add(self, waypoint, index = 'unspecified'):
        """
        :param waypoint: waypoint to add
        :type waypoint: list
        """
        if index is 'unspecified':
            self.waypoints.append(waypoint)
        else:
            self.waypoints.insert(index, waypoint)

    #delete waypoint at the index
    def delete(self, index):
        """
        :param index: index to delete
        :type index: int
        """
        del self.waypoints[index]

    #return waypoint at current index and increment index with wrapping
    def next(self):
        output = self.waypoints[self.index]
        self.index += 1
        if self.index is len(self.waypoints):
            self.index = 0
        return output

    #change index and return waypoint at new index
    def goto(self, index):
        """
        :param index: index to delete
        :type index: int
        """
        self.index = index
        return self.waypoints[index]

    #return length
    def length(self):
        return len(self.waypoints)
