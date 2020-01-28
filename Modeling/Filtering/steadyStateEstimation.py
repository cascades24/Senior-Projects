############################################################################################
#
# Title: Steady State Estimation
#
# Purpose: Kalman filtering of a steady state particle at x=10, y=10.
#
# Author: Adam Farmer
#
# Date Written:  1/23/19
# Date Modified: 1/25/19
#
############################################################################################

## Import libraries (if needed)

import numpy as np
import matplotlib.pyplot as plt

## Initial conditions and constants

# Initial position and velocity, [x_i , xDot_i]
stateVec_i = np.array([10.0, 10.0, 0.0, 0.0])

# Time stuff
t0 = 0 # Seconds
dt = .1 # Seconds
tf = 10.0 # Seconds
# Make time vector:
tVec = np.linspace(t0, tf, ((tf-t0)/dt))

# Preallocate state and change in state vectors:
stateVec = np.zeros((4, len(tVec)))
stateVecFiltered = np.zeros((4, len(tVec)))

P_p = np.zeros((4, 4, len(tVec)))

## Function definitions


def truncNormalDist(trunc, mean, sigma):

    # Simulate random noise sampled from normal distribution centered around 'mean', with stddev 'sigma' and truncate
    # at 'trunc'.

    while True:
        noise = np.random.normal(mean, sigma)
        if abs(noise) <= trunc:
            break

    return noise


def kalmanFilter(x_p0, x_pk0, F_k0, G_k0, Q_k0, R_k1, H_k1, y_k1, u_k0, I, P_pk0):

    # Matrices in this function attempt to closely follow notation in Optimal State Estimation by Simon, pg.128
    # The following is clarification for variables names in this function:
    # p = plus,  m = minus,  0 = k-1,  1 = k

    # Extrapolation to the next time-step, no filtering yet
    P_mk1 = np.add(np.matmul(F_k0, np.matmul(P_pk0, np.transpose(F_k0))), Q_k0)
    # Kalman filter gain:
    K_k1 = np.matmul(P_mk1, np.matmul(np.transpose(H_k1), np.linalg.inv(np.add(np.matmul(H_k1,
                                                                        np.matmul(P_mk1, np.transpose(H_k1))), R_k1))))
    x_mk1 = np.add(np.matmul(F_k0, x_pk0), np.matmul(G_k0, u_k0))

    # Update the estimate of the state at the new time-step, now filtered
    x_pk1 = x_mk1 + np.matmul(K_k1, (y_k1 - np.matmul(H_k1, x_mk1)))
    P_pk1 = np.matmul((I - np.matmul(K_k1, H_k1)), np.matmul(P_mk1, np.transpose(I - np.matmul(K_k1, H_k1))))  \
            + np.matmul(K_k1, np.matmul(R_k1, np.transpose(K_k1)))

    return [x_pk1, P_pk1]


## Do the stuff:

# Pre-allocation for noise vectors:
v_k = np.array([0.0, 0.0, 0.0, 0.0])
w_k = np.array([0.0, 0.0, 0.0, 0.0])

# Initialize covariance matrices for dynamics and sensors, respectively:
Q_k0 = np.array([[0.8, 0.0, 0.0, 0], [0.0, 0.8, 0.0, 0.0], [0.0, 0.0, 1.2, 0.0], [0.0, 0.0, 0.0, 1.2]])
R_k0 = np.array([[0.8, 0.0, 0.0, 0], [0.0, 0.8, 0.0, 0.0], [0.0, 0.0, 1.2, 0.0], [0.0, 0.0, 0.0, 1.2]])*10

# State transition matrix:
F_k0 = np.eye(4)

# Control vector and control dynamics matrix:
u_k0 = np.array([0, 0, 0, 0])
G_k0 = np.zeros((4, 4))

# Measurements, currently all state variables are measured:
H_k1 = np.eye(4)

# Initialize Kalman Filter covariance matrix:
P_pk0 = np.array([[10.0, 0.0, 0.0, 0.0], [0.0, 10.0, 0.0, 0.0], [0.0, 0.0, 10.0, 0.0], [0.0, 0.0, 0.0, 10.0]])*0
X0 = np.array([0, 0, 0, 0])

for index, ti in enumerate(tVec[:-1]):

    # Put in initial conditions:
    if index is 0:
        stateVec[:, index] = stateVec_i
        stateVecFiltered[:, index] = X0

    # Get truth data:
    stateVec[:, index+1] = stateVec_i

    # Simulate process noise as sampling from truncated, random, normal distribution:
    for i in range(0, 4):
        v_k[i] = truncNormalDist(3, 0, R_k0[i, i])
        w_k[i] = truncNormalDist(3, 0, Q_k0[i, i])

    # Output vector. what if this is ones + noise?
    y_k1 = np.add(np.matmul(H_k1, stateVec[:, index]), v_k)

    # Save the covariance matrix to look at it later:
    P_p[:, :, index] = P_pk0

    # Kalman filtering:
    [stateVecFiltered[:, index+1], P_pk0] = kalmanFilter(X0, stateVecFiltered[:, index], F_k0, G_k0, Q_k0,
                                                         R_k0, H_k1, y_k1, u_k0, np.eye(4), P_pk0)


## Plotting

plt.figure(1)
plt.minorticks_on()
plt.subplot(2, 1, 1)
plt.plot(tVec[:-1], stateVec[0, :-1], '--k')
plt.plot(tVec[:-1], stateVecFiltered[0, :-1], 'c')
plt.title('Position Components VS Time')
plt.legend(['Dynamics', 'Filtered Data'])
plt.ylabel('X position [m]')
plt.grid(which='major', color='k', linestyle='-', linewidth=.1)
plt.grid(which='minor', color='k', linestyle=':', linewidth=.1)
plt.subplot(2, 1, 2)
plt.plot(tVec[:-1], stateVec[1, :-1], '--k')
plt.plot(tVec[:-1], stateVecFiltered[1, :-1], 'c')
plt.legend(['Dynamics', 'Filtered Data'])
plt.xlabel('Time (seconds)')
plt.ylabel('Y position [m]')
plt.grid(which='major', color='k', linestyle='-', linewidth=.1)
plt.grid(which='minor', color='k', linestyle=':', linewidth=.1)


plt.figure(2)
plt.minorticks_on()
plt.subplot(4, 1, 1)
plt.plot(tVec[:-1], P_p[0, 0, :-1], 'c')
plt.title('Covariance VS Time')
plt.legend(['P1'])
plt.ylabel('P_p[1,1]')
plt.grid(which='major', color='k', linestyle='-', linewidth=.1)
plt.grid(which='minor', color='k', linestyle=':', linewidth=.1)
plt.subplot(4, 1, 2)
plt.plot(tVec[:-1], P_p[1, 1, :-1], 'c')
plt.legend(['P2'])
plt.xlabel('Time (seconds)')
plt.ylabel('P_p[1,1]')
plt.grid(which='major', color='k', linestyle='-', linewidth=.1)
plt.grid(which='minor', color='k', linestyle=':', linewidth=.1)
plt.subplot(4, 1, 3)
plt.plot(tVec[:-1], P_p[2, 2, :-1], 'c')
plt.legend(['P3'])
plt.xlabel('Time (seconds)')
plt.ylabel('P_p[1,1]')
plt.grid(which='major', color='k', linestyle='-', linewidth=.1)
plt.grid(which='minor', color='k', linestyle=':', linewidth=.1)
plt.subplot(4, 1, 4)
plt.plot(tVec[:-1], P_p[3, 3, :-1], 'c')
plt.legend(['P3'])
plt.xlabel('Time (seconds)')
plt.ylabel('P_p[1,1]')
plt.grid(which='major', color='k', linestyle='-', linewidth=.1)
plt.grid(which='minor', color='k', linestyle=':', linewidth=.1)

plt.show()
