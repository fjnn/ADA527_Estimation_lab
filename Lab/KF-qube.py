# https://www.diva-portal.org/smash/get/diva2:1763450/FULLTEXT01.pdf
# 2.1.3 Linearised State Equations


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from kalman_filter import KalmanFilter

# PARAMETERS

# Motor
# Resistance
Rm = 7.5
# Current-torque (N-m/A)
kt = 0.042
# Back-emf constant (V-s/rad)
km = 0.042

# Rotary Arm
# Mass (kg)
mr = 0.095
# Total length (m)
r = 0.085
# Moment of inertia about pivot (kg-m^2)
Jr = mr*(r**2)/3
# Equivalent Viscous Damping Coefficient (N-m-s/rad)
br = 1e-3 # damping tuned heuristically to match QUBE-Sero 2 response

# Pendulum Link
# Mass (kg)
mp = 0.024
# Total length (m)
Lp = 0.129
# Pendulum center of mass (m)
l = Lp/2
# Moment of inertia about pivot (kg-m^2)
Jp = mp*(Lp**2)/3
# Equivalent Viscous Damping Coefficient (N-m-s/rad)
bp = 5e-5 # damping tuned heuristically to match QUBE-Sero 2 response
# Gravity Constant
g = 9.81

# Moment of inertia of pendulum about center of mass (kg-m^2)
Jp_cm = mp*(Lp**2)/12 # used to calculate pendulum energy in swing-up control


theta = 0
alpha = 0
theta_dot = 0
alpha_dot = 0
state_vector = np.array([theta, alpha, theta_dot, alpha_dot]).T

observation_vector = np.array([theta, alpha]).T

A = np.array([[0,0,1,0],
              [0,0,0,1],
              [0,mp**2*l**2*r*g, -Jp*br, mp*l*r*bp],
              [0, -mp*g*l*Jr, mp*l*r*br, -Jp*bp]])

B = np.array([0, 0, Jp, -mp*r*l]).T
# Torque is input
torque_input = np.zeros((100, 1), np.float32)

C = np.array([[1,0,0,0], [0,1,0,0]])
D = np.array([0,0])


estimateCovariance = np.eye(state_vector.shape[0])
transitionMatrix = A
processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], np.float32) * 0.000005
observationMatrix = C
measurementNoiseCov = np.array([[1,0],[0,1]], np.float32) * 1
controlMatrix = B

kalman = KalmanFilter(X=state_vector,
                      P=estimateCovariance,
                      F=(1/Jp)*transitionMatrix,
                      Q=processNoiseCov,
                      B=(1/Jp)*B,
                      M=torque_input,
                      Z=observation_vector,
                      H=observationMatrix,
                      R=measurementNoiseCov)

print("done")