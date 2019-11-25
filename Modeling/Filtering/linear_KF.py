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

	a = np.loadtxt('stationary.txt', delimiter=',') #x,y,z
	#t = np.loadtxt('timestamp.txt', delimiter=', ')
	t = np.linspace(0,8,115)
	w_noise = np.random.rand(1,6)*0.1
	m_noise = np.random.rand(1,6)*0.1
	ts = t[1]
	position = np.array(a[0,:]*ts*ts)
	velocity = np.array(a[0,:]*ts)
	#B = [0.1 , 0 , 0 , 0]
	x = [0,0,0,0,0,0]
	P = np.eye(6)*np.random.rand(1,1)*100 #Initial uncertainty
	A = np.array([[1, 0, 0, ts, 0, 0],
                 [0, 1, 0, 0, ts, 0],
                 [0, 0, 1, 0, 0, ts],
                 [0, 0, 0, 1, 0, 0],
                 [0, 0, 0, 0, 1, 0],
                 [0, 0, 0, 0, 0, 1]])
	Q = np.eye(6)*0.75 #actuator type uncertainy 1%


	H = np.eye(6)

	R = np.eye(6)*100 

	pose, P = kf(x,t,P,A,Q,H,R,a,w_noise,m_noise)

	stdx = np.sqrt(P[:,0,0])
	stdy = np.sqrt(P[:,1,1])
	stdz = np.sqrt(P[:,2,2])



	fig, axs = plt.subplots(2, 2)
	fig.suptitle("Kalman Filtered data for Beacon at (10,10,10)", fontsize=34)
	#fig.legend('x', 'y', 'z')
	#Truth data
	truth = axs[0,0].hlines(10,0,8, linewidth = 2,linestyle = '--', color='black')
	axs[0,1].hlines(10,0,8, linewidth = 2,linestyle = '--', color='black')
	axs[1,0].hlines(10,0,8, linewidth = 2,linestyle = '--', color='black')



	filtered, = axs[0,0].plot(t,pose[:,0], color='blue')
	#plt.plot(t[:,0],P[:,0,0])
	axs[0,1].plot(t,pose[:,1], color='blue', label='std')
	axs[1,0].plot(t,pose[:,2], color='blue')


	raw, = axs[0,0].plot(t,a[:,0], color='green')
	axs[0,1].plot(t,a[:,1], color='green')
	axs[1,0].plot(t,a[:,2], color='green')

	axs[1,1].plot(t,P[:,0,0], color='green')
	axs[1,1].plot(t,P[:,1,1], color='blue')
	axs[1,1].plot(t,P[:,2,2], color='red')
	
	#2 sigma bounds
	sigma = axs[0,0].fill_between(t,stdx + pose[:,0],pose[:,0] - stdx, color='#CED8FF')
	axs[0,1].fill_between(t,stdy + pose[:,1],pose[:,1] - stdx, color='#CED8FF')
	axs[1,0].fill_between(t,stdz + pose[:,2],pose[:,2] - stdx, color='#CED8FF')

	labels = ["Truth", "Raw", "Filtered", "sigma"]
	fig.legend([truth, raw, filtered, sigma], labels, loc = (0.9, 0.5))

	axs[0, 0].set_title('X',fontsize=24)
	axs[0, 1].set_title('Y',fontsize=24)
	axs[1, 0].set_title('Z',fontsize=24)
	axs[1, 1].set_title('Covariance', fontsize=24)

	axs[0,0].set_xlabel('time [s]', fontsize=20)
	axs[0,0].set_ylabel('distance [m]', fontsize=20)
	axs[0,1].set_xlabel('time [s]', fontsize=20)
	axs[0,1].set_ylabel('distance [m]', fontsize=20)
	axs[1,0].set_xlabel('time [s]', fontsize=20)
	axs[1,0].set_ylabel('distance [m]', fontsize=20)
	axs[1,1].set_xlabel('time [s]', fontsize=20)


	axs[1,1].legend(('x', 'y', 'z'))
	axs[0,0].grid()
	axs[0,1].grid()
	axs[1,0].grid()
	axs[1,1].grid()

	plt.show()



def kf(xp,t,Pp,A,Q,H,R,a, w,m): 
	dt = t[1] # discrete
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

		#if k>2:
		#	z[3] = abs(x[k,0]-x[k-1,0])/dt
		#	z[4] = abs(x[k,1]-x[k-1,1])/dt
		#	z[5] = abs(x[k,2]-x[k-1,2])/dt
		#	print z[3]
		x[k+1] = xkp + np.dot(K,(z-np.dot(H,xkp))) 

		P[k+1] = np.dot((np.eye(6) - np.dot(K,H)),pkp)
		
	return x, P


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
