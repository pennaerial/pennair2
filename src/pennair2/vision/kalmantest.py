import kalman as kal
import numpy as np
from matplotlib import pyplot as plt
from filterpy.common import Q_discrete_white_noise

dt = 0.01
x = np.arange(0,30,dt)
y = np.sin(0.5*x)
v = 0.5*np.cos(0.5*x)
ynoisey = y + 0.05*np.sin(20*x)+0.1*np.cos(8*x)

kf = kal.Kalman(F="pva",dt=dt,H=0,Q=0.8,R=5000)
output = []
for i in range(x.size):
	output.append(kf.predict([ynoisey[i]])[0])

plt.plot(x,ynoisey,x,output)
plt.show()