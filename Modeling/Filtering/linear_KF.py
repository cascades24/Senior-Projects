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

	a = np.loadtxt('myData.txt', delimiter=',') #x,y,z
	t = np.loadtxt('timestamp.txt', delimiter=', ')
	w_noise = np.random.rand(1,6)*0.1
	m_noise = np.random.rand(1,6)*0.1
	ts = t[1,0]
	position = np.array(a[0,:]*ts*ts)
	velocity = np.array(a[0,:]*ts)

	x = [0,0,0,0,0,0]
	P = np.eye(6)*np.random.rand(1,1) #Initial uncertainty
	A = np.array([[1, 0, 0, ts, 0, 0],
                 [0, 1, 0, 0, ts, 0],
                 [0, 0, 1, 0, 0, ts],
                 [0, 0, 0, 1, 0, 0],
                 [0, 0, 0, 0, 1, 0],
                 [0, 0, 0, 0, 0, 1]])
	Q = np.eye(6)*0.25 #actuator type uncertainy 1%


	H = np.eye(6)

	R = np.eye(6)*1 #noise 0.1 rad

	pose, P = kf(x,t,P,A,Q,H,R,a,w_noise,m_noise)

	plt.title('data')
	plt.plot(t[:,0],pose[:,0])
	#plt.plot(t[:,0],P[:,0])
	#plt.plot(t[:,0],pose[:,1])
	#plt.plot(t[:,0],pose[:,2])


	plt.plot(t[:,0],a[:,0])
	#plt.plot(t[:,0],a[:,1]*timestep*timestep)
	#plt.plot(t[:,0],a[:,2]*timestep*timestep)

	plt.xlabel('time (sec)')
	plt.ylabel('m')


	plt.legend(('x', 'y', 'z'))
	plt.grid()
	plt.show()



def kf(xp,t,Pp,A,Q,H,R,a, w,m): 
	dt = t[1,0] # discrete
	x = np.zeros((len(t),6))
	x[0,:] = xp
	P = np.zeros((len(t),6,6))
	P[0] = Pp

	for k in range(0,len(t)-1):
		#Predict
		xkp = np.dot(A,(x[k])) 

		pkp = np.dot(A,np.dot(P[k],np.transpose(A))) + Q

		#Correct
		K = np.dot(np.dot(pkp, np.transpose(H)), np.linalg.inv(np.dot(np.dot(H,pkp),np.transpose(H))+ R))

		z = [a[k+1,0],a[k+1,1],a[k+1,2],0,0,0]

		x[k+1] = xkp + np.dot(K,(z-np.dot(H,xkp))) 

		P[k+1] = np.dot((np.eye(6) - np.dot(K,H)),pkp)
		
	return x, P


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
