#!/usr/bin/env python2
import rospy
import numpy as np
import sys,os, os.path
import math
import matplotlib.pyplot as plt
import yaml

def main():
	#dynamics = sys.argv[1] #Next semester AKA deployment
	#with open(dynamics, 'r') as fp:
	#	dynamics = yaml.load(fp)

	#x = dynamics['initial_state']
	#t = dynamics['timestep']

	a = np.loadtxt('accel.txt', delimiter=', ') #x,y,z
	t = np.loadtxt('timestamp.txt', delimiter=', ')
	#w_noise = np.random.rand(6,1)*0.001
	#m_noise = np.random.rand(6,1)*0.001
	timestep = t[1,0]
	position = np.array(a[0,:]*timestep*timestep)
	velocity = np.array(a[0,:]*timestep)

	x = np.concatenate((position,velocity),axis=0) #Initial state [x,y,z,Vx,Vy,Vz]

	P = np.eye(6)*np.random.rand(1,1) #Initial uncertainty
	A = np.eye(6)
	Q = np.eye(6)*0.001 #actuator type uncertainy 1%


	H = np.eye(6)

	R = np.eye(6)*0.1 #noise 0.1 rad

	pose = kf(x,t,P,A,Q,H,R,a)
	print len(t)
	plt.title('data')
	plt.plot(t[:,0],pose[:,0])
	plt.plot(t[:,0],pose[:,1])
	plt.plot(t[:,0],pose[:,2])
	plt.xlabel('time (sec)')
	plt.ylabel('m')


	plt.legend(('x', 'y', 'z'))
	plt.grid()
	plt.show()



def kf(xp,t,P,A,Q,H,R,a): 
	timestep = t[1,0] # discrete
	x = np.zeros((len(t)+1,6))
	x[0] = xp
	for k in range(1,len(t)):

		#Predict
		xkp = np.dot(A,np.transpose(x[k]))
		pkp = np.dot(A,np.dot(P,np.transpose(A))) + Q

		#Correct
		K = np.dot(np.dot(pkp, np.transpose(H)), np.linalg.inv(np.dot(np.dot(H,pkp),np.transpose(H))+ R))

		z = [a[k,0]*timestep*timestep,a[k,1]*timestep*timestep, a[k,2]*timestep*timestep, a[k,0]*timestep,a[k,1]*timestep,a[k,2]*timestep]


		x[k+1] = xkp + np.dot(K,(z-np.dot(H,xkp)))

		P = np.dot((np.eye(6) - np.dot(K,H)),pkp)
		
	return x


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
