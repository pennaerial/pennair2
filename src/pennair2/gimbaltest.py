import unittest
from gimbal import Gimbal
from pyquaternion import Quaternion
from servo import Servo
from math import pi
import numpy as np
import math

class GimbalTest(unittest.TestCase):

	def setUp(self):
		pass

	def test_simple_vector(self):
		g = Gimbal(None, None, None, 0, True)
		self.assertEqual(g.gimbal_set_vector([1, 0, 0]), (0, 0, pi/2))
	
	def test_xy_vector(self):
		g = Gimbal(None, None, None, 0, True)
		tup1 = g.gimbal_set_vector([1, 1, 0])
		tup2 = (pi/4, 0, pi/2)
		self.assertAlmostEqual(tup1[0], tup2[0], places=15) 
		self.assertAlmostEqual(tup1[1], tup2[1], places=15) 
		self.assertAlmostEqual(tup1[2], tup2[2], places=15) 
	
	def test_3d_vector(self):
		g = Gimbal(None, None, None, 0, True)
		tup1 = g.gimbal_set_vector([1, 1, 1])
		tup2 = (pi/4, pi/4, pi/4)
		self.assertAlmostEqual(tup1[0], tup2[0], places=15) 
		self.assertAlmostEqual(tup1[1], tup2[1], places=15) 
		self.assertAlmostEqual(tup1[2], tup2[2], places=15) 

	def test_simple_point_in_space(self):
		g = Gimbal(None, None, None, 0, True)
		tup1 = g.gimbal_point_in_space([0, 0, 0], [1, 1, 1], None)
		tup2 = (pi/4, pi/4, pi/4)
		self.assertAlmostEqual(tup1[0], tup2[0], places=15) 
		self.assertAlmostEqual(tup1[1], tup2[1], places=15) 
		self.assertAlmostEqual(tup1[2], tup2[2], places=15) 

	def test_quaternion_do_nothing(self):
		g = Gimbal(None, None, None, 0, True)
		q = Quaternion(axis=[1, 0, 0], angle=0)
		tup1 = g.gimbal_point_in_space([0, 0, 0], [1, 1, 1], q)
		tup2 = (pi/4, pi/4, pi/4)
		self.assertAlmostEqual(tup1[0], tup2[0], places=15) 
		self.assertAlmostEqual(tup1[1], tup2[1], places=15) 
		self.assertAlmostEqual(tup1[2], tup2[2], places=15) 
	
	def test_has_distance(self):
		g = Gimbal(None, None, None, 1, True)
		tup1 = g.gimbal_point_in_space([0, 0, 0], [1, 0, 0], None)
		tup2 = g.gimbal_set_vector([1, 0, 1])
		self.assertEqual(tup1, tup2)

	def test_quaternion_full_rotation(self):
		g = Gimbal(None, None, None, 0, True)
		q = Quaternion(axis=[1, 0, 0], angle=pi)
		tup1 = g.gimbal_point_in_space([0, 0, 0], [1, 1, 1], q)
		tup2 = (pi/4, pi/4, pi/4)
		self.assertAlmostEqual(tup1[0], tup2[0], places=15) 
		self.assertAlmostEqual(tup1[1], tup2[1], places=15) 
		self.assertAlmostEqual(tup1[2], tup2[2], places=15) 

	def test_quaternion_half_rotation(self):
		g = Gimbal(None, None, None, 0, True)
		q = Quaternion(axis=[1, 0, 0], angle=pi/2)
		tup1 = g.gimbal_point_in_space([0, 0, 0], [1, 1, 1], q)
		tup2 = (pi/4, pi/4, pi/4)
		self.assertAlmostEqual(tup1[0], tup2[0], places=15) 
		self.assertAlmostEqual(tup1[1], tup2[1], places=15) 
		self.assertAlmostEqual(tup1[2], tup2[2], places=15)

if __name__ == '__main__':
    unittest.main()