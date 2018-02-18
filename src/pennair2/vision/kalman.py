import numpy as np
import time
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

# https://github.com/rlabbe/filterpy/blob/master/filterpy/kalman/kalman_filter.py

class Kalman:

	def __init__(self,F='pv',H=None,dt=0,numsensors=1,Q=None,R=None,P=None,B=None,initialstate=None,**kwargs):
		# Filter Init
		if type(F)==str:
			self.states = len(F)
		else:
			self.states = F.shape[0]
		self.filter = KalmanFilter(dim_x=self.states,dim_z=numsensors)
		self.initialstate = initialstate
		if initialstate is not None:
			self.filter.x = np.array(initialstate)
		# Time Init
		self.autotime = (dt == 0)
		self.dt = dt
		if self.autotime:
			self.tic = time.time()
		# F (Next State Prediction) Init size:(states,states)
		self.typeF = None
		if type(F)==str and (('p' in F) or ('v' in F) or ('a' in F)):
			if (F == 'pva'):
				self.typeF = 0
			elif (F == 'pv') or (F == 'va'):
				self.typeF = 1
			elif (F == 'pa'):
				self.typeF = 2
			else:
				self.typeF = 3
		self.recalcF = (self.typeF is not None)
		if self.recalcF:
			self.calcF()
		else:
			self.filter.F = np.array(F)
		# H Measurement Init size:(states,states) (Give matrix, or tuple (1,3) to provide Hs for only states 1 and 3)
		if type(H)==list:
			self.filter.H = np.array(H)
		else:
			self.filter.H = np.identity(self.states)
			if type(H)==int or type(H)==tuple:
				self.filter.H = self.filter.H[H,:]
			if len(self.filter.H.shape)==1:
				self.filter.H = np.array([self.filter.H])
		# P Covariance Init size:(states,states)
		if P is None:
			self.filter.P *= 1000
		elif type(P)==int or type(P)==float:
			self.filter.P *= P
		else:
			self.filter.P = np.array(P)
		# R Sensor Uncertainty Init size:(numsensors,numsensors)
		if R is None:
			self.filter.R = 5
		elif type(R)==int or type(R)==float:
			self.filter.R = R
		else:
			self.filter.R = np.array(R)
		# Q Actual Volatility Init size:(states,states)
		self.recalcQ = (type(Q)!=np.ndarray)
		if self.recalcQ:
			if (type(Q)==int) or (type(Q)==float):
				self.Qmag = Q
			elif type(self.filter.R)==np.ndarray:
				self.Qmag = np.linalg.norm(self.filter.R)
			else:
				self.Qmag = self.filter.R
			self.calcQ()
		else:
			self.filter.Q = np.array(Q)
		# B Control Init (Extra Forces)
		if B is not None:
			self.filter.B = np.array(B)
		# Other Init
		for name, val in kwargs.iteritems():
			if name in self.filter.__dict__:
				self.filter.__dict__[name] = val


	def calcDt(self):
		if self.autotime:
			self.dt = time.time() - self.tic
			self.tic = time.time()

	def calcF(self):
		if self.recalcF:
			if self.typeF == 0:
				self.filter.F = np.array([[1, self.dt, 0.5*self.dt**2], [0, 1, self.dt],[0, 0, 1]])
			elif self.typeF == 1:
				self.filter.F = np.array([[1, self.dt], [0, 1]])
			elif self.typeF == 2:
				self.filter.F = np.array([[1, 0.5*self.dt^2], [0,1]])
			elif self.typeF == 3:
				self.filter.F = np.array([1])

	def calcQ(self):
		if self.recalcQ:
			self.filter.Q = Q_discrete_white_noise(self.states, self.dt, self.Qmag)

	# xex: x expected, xme: m measured, xf: x filtered, P: covariance, H: measurement, F: prediction matrix K: kalman constant
	# xex = (F * xf) + (B * u)
	# P = (F * P * F') + R
	# K = P * H' * inv(H*P*H' + Q)
	# xf = xex + K*(xme - H*xex)
	# P = (I - K*H) * P
	def predict(self,x=None):
		self.calcDt()
		self.calcF()
		self.calcQ()
		if x is not None:
			x = np.array([x])
			x = x.reshape((x.size,1))
			if self.initialstate is None:
				self.initialstate = x
				self.filter.x = np.matmul(np.transpose(self.filter.H),self.initialstate)
		self.filter.predict()
		if x is not None:
			self.filter.update(x)
		return self.filter.x


