#!/usr/bin/env python2
import rospy
import numpy as np
import sys,os, os.path
import math
import matplotlib.pyplot as plt


def main():
	data_type = sys.argv[1]
	a = np.loadtxt(data_type + '/accel.txt', delimiter=', ') #x,y,z
	g = np.loadtxt(data_type + '/gyro.txt', delimiter=', ') # p,q,r
	m = np.loadtxt(data_type + '/mag.txt', delimiter=', ') #Heckman said ignore
	t = np.loadtxt(data_type + '/timestamp.txt', delimiter=', ')


	angles = kf(a,g,m,t)
	plt.title(data_type)
	plt.plot(t[:,0],angles[:,0])
	plt.plot(t[:,0],angles[:,1])
	plt.plot(t[:,0],angles[:,2])
	plt.xlabel('time (sec)')
	plt.ylabel('rad')


	plt.legend(('phi', 'theta', 'psi'))
	plt.grid()
	plt.show()



def kf(a,g,m,t): #For the given problem we only really need to track rotation states
	gx = g[:,0]
	gy = g[:,1]
	gz = g[:,2]
	t = t[:,0]
	x = np.zeros((len(t)+1,3))
	angles = np.zeros((len(t),3))

	timestep = t[0] - t[1] # discrete

	#Initialize
	x[0] = [gx[0]*timestep,gy[0]*timestep, gz[0]*timestep] #first state

	#w_noise = np.random.rand(6,1)*0.001
	#m_noise = np.random.rand(6,1)*0.001

	P = np.eye(3)*np.random.rand(1,1) #Initial uncertainty
	A = np.eye(3)
	Q = np.eye(3)*0.001 #actuator type uncertainy 1%


	H = np.array([[ 1, 0, 0], [ 0, 1, 0], [ 0, 0, 1]])  #Maybe this

	R = np.eye(3)*0.1 #noise 0.1 rad

	for k in range(1,len(t)):
		#Update formula
		part = np.multiply(euler(x[k-1]),timestep)
		angles[k] = np.dot(part,np.multiply(x[k-1],1/timestep)) + angles[k-1]

		#Predict
		xkp = np.dot(A,x[k])
		pkp = np.dot(A,np.dot(P,np.transpose(A))) + Q

		#Correct
		K = np.dot(np.dot(pkp, np.transpose(H)), np.linalg.inv(np.dot(np.dot(H,pkp),np.transpose(H))+ R))
		z = [gx[k]*timestep,gy[k]*timestep, gz[k]*timestep]

		x[k+1] = xkp + np.dot(K,(z-np.dot(H,xkp)))
		print x[k+1]

		P = np.dot((np.eye(3) - np.dot(K,H)),pkp)
		
	return angles

def euler(x):
    return [[1, math.sin(x[0]) * math.tan(x[1]), math.cos(x[0]) * math.tan(x[1])],
             [0, math.cos(x[0]), -math.sin(x[0])],
             [0, math.sin(x[0]) / math.cos(x[1]), math.cos(x[0]) / math.cos(x[1])]]


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
