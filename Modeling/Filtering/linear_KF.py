#!/usr/bin/env python2
import rospy
import numpy as np
import sys,os, os.path
import math
import matplotlib.pyplot as plt
import yaml

def main():
	dynamics = sys.argv[1]


	a = np.loadtxt(dynamics, delimiter=',') #x,y,z
	#t = np.loadtxt('timestamp.txt', delimiter=', ')
	t = np.linspace(0,8,115)
	ts = t[1]
	position = np.array(a[0,:]*ts*ts)
	velocity = np.array(a[0,:]*ts)
	


	x0 = [0,0,0,0,0,0]
	x = np.zeros((len(t),6))
	x[0,:] = x0
	P = np.zeros((len(t),6,6))
	P[0,:,:] = np.eye(6)*np.random.rand(1,1)*300 #Initial uncertainty

	dt = t[0] # discrete

	A = np.array([[1, 0, 0, dt, 0, 0],
                 [0, 1, 0, 0, dt, 0],
                 [0, 0, 1, 0, 0, dt],
                 [0, 0, 0, 1, 0, 0],
                 [0, 0, 0, 0, 1, 0],
                 [0, 0, 0, 0, 0, 1]])
	Q = np.eye(6)*2.75 #actuator type uncertainy 1%


	H = np.eye(6)

	R = np.array([[100, 0, 0, 0, 0, 0],
                 [0, 100, 0, 0, 0, 0],
                 [0, 0, 100, 0, 0, 0],
                 [0, 0, 0, 400, 0, 0],
                 [0, 0, 0, 0, 400, 0],
                 [0, 0, 0, 0, 0, 400]])

	if dynamics == 'stationary.txt':
		for k in range(0,len(t)-1):
			x[k+1], P[k+1] = linear_kf_stationary(k, a[k+1],t[k], x[k], P[k], a[k], A,Q,H,R)
	else: 
		for k in range(0,len(t)-1):
			x[k+1], P[k+1] = linear_kf_motion(k, a[k+1],t[k], x[k], P[k], a[k], A,Q,H,R)
	
	plot(x,P,t,a)

def plot(pose, P,t,a):

	stdx = np.sqrt(P[:,0,0])
	stdy = np.sqrt(P[:,1,1])
	stdz = np.sqrt(P[:,2,2])



	fig, axs = plt.subplots(2, 2)
	fig.suptitle("Kalman Filtered data for Beacon in cluster motion", fontsize=34)
	#fig.legend('x', 'y', 'z')
	#Truth data
	y = np.zeros((len(t),1))
	z = np.zeros((len(t),1))
	for i in range(0,len(t)):
		y[i] = 10 +0.1*i
		z[i] = 10 -0.1*i
	axs[0,0].plot(t,y, linewidth = 2,linestyle = '--', color='black')
	axs[0,1].plot(t,z, linewidth = 2,linestyle = '--', color='black')
	truth = axs[1,0].hlines(10,0,8, linewidth = 2,linestyle = '--', color='black')



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


def linear_kf_motion(k, a,t, x, P, prev_z, A, Q ,H ,R): 	

	#Predict
	xkp = np.dot(A,(x)) 

	pkp = np.dot(A,np.dot(P,np.transpose(A))) + Q

	#Correct
	K = np.dot(np.dot(pkp, np.transpose(H)), np.linalg.inv(np.dot(np.dot(H,pkp),np.transpose(H))+ R))


	if t != 0:
		Vx = (a[0] - prev_z[0])/t
		Vy = (a[1] - prev_z[1])/t
		Vz = (a[2] - prev_z[2])/t
	else:
		Vx = 0
		Vy = 0
		Vz = 0 

	z = [a[0],a[1],a[2],Vx, Vy, Vz]
	
	x = xkp + np.dot(K,(z-np.dot(H,xkp))) 

	P = np.dot((np.eye(6) - np.dot(K,H)),pkp)
		
	return x, P


def linear_kf_stationary(k, data,t, x, P, prev_z, A, Q ,H ,R): 	

	#Predict
	xkp = np.dot(A,(x)) 

	pkp = np.dot(A,np.dot(P,np.transpose(A))) + Q

	#Correct
	K = np.dot(np.dot(pkp, np.transpose(H)), np.linalg.inv(np.dot(np.dot(H,pkp),np.transpose(H))+ R))

	z = [a[0],a[1],a[2],0,0,0]

	x = xkp + np.dot(K,(z-np.dot(H,xkp))) 

	P = np.dot((np.eye(6) - np.dot(K,H)),pkp)
		
	return x, P


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
