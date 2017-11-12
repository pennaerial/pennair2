import unittest
from gimbal import *
from math import pi
import numpy as np

class GimbalTest(unittest.TestCase):

	def setUp(self):
		pass

	def test_simple_vector(self):
		g = Gimbal(None, None, None, 0)
		self.assertEqual(g.gimbal_set_vector([1, 0, 0]), (0, pi/2, pi/2))


	def test_xy_vector(self):
		g = Gimbal(None, None, None, 0)
		np.testing.assert_almost_equal(g.gimbal_set_vector([1, 1, 0]), (pi/4, pi/4, pi/2), 0.0000001) 

	def test_3d_vector(self):
		g = Gimbal(None, None, None, 0)
		np.testing.assert_almost_equal(g.gimbal_set_vector([1, 1, 1]), (pi/4, pi/4, pi/4), 0.0000001) 


def main():
	unittest.main()

if __name__ == '__main__':
    main()