import unittest
import waypoint

class TestWaypoint(unittest.TestCase):

    def test_contructor(self):
        points = waypoint.Waypoints("euclidean")
        self.assertEqual(points.waypoints, [])
        self.assertEqual(points.index, 0)

    def test_add(self):
        points = waypoint.Waypoints("euclidean")
        points.add([0, 0, 0])
        self.assertEqual(points.waypoints, [[0, 0, 0]])
        points.add([1, 1, 1])
        self.assertEqual(points.waypoints, [[0, 0, 0], [1, 1, 1]])
        points.add([2, 2, 2])
        self.assertEqual(points.waypoints, [[0, 0, 0], [1, 1, 1], [2, 2, 2]])
        points.add([3, 3, 3])
        self.assertEqual(points.waypoints, [[0, 0, 0], [1, 1, 1], [2, 2, 2],
                                            [3, 3, 3]])
        points.add([4, 4, 4], 0)
        self.assertEqual(points.waypoints, [[4, 4, 4], [0, 0, 0], [1, 1, 1],
                                            [2, 2, 2], [3, 3, 3]])

    def test_delete(self):
        points = waypoint.Waypoints("euclidean")
        points.add([0, 0, 0])
        points.add([1, 1, 1])
        points.add([2, 2, 2])
        points.add([3, 3, 3])
        points.delete(1)
        self.assertEqual(points.waypoints, [[0, 0, 0], [2, 2, 2], [3, 3, 3]])
        points.delete(0)
        self.assertEqual(points.waypoints, [[2, 2, 2], [3, 3, 3]])
        points.delete(1)
        self.assertEqual(points.waypoints, [[2, 2, 2]])
