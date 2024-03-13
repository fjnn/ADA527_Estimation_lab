import numpy as np
import sympy as sym
from spatialmath import *
import roboticstoolbox as rtb
from roboticstoolbox import ET, Link
from math import radians as d2r

# Define the transformations for qube and camera

l1 = 350 # camera-qube distance (x-axis)
l2 = 160 # camera height (z-axis)
l3 = 127 # motor height (z-axis)
l4 = 90 # pendulum offset (x-axis)
l5 = 65 # half of rod lenght (-z-axis(!))

## Camera transformation
T_w_camera = Link(ET.tz(l1) * ET.Rz(np.pi))

## Qube+Pendulum is like the second robot
theta = d2r(0)
alpha = d2r(0)
# T_w_motor = Link(ET.tz(l3) * ET.Rz(theta))
# T_motor_rodCOM = Link(ET.Rz(0) * ET.tx(l4) * ET.ty(l5*np.sin(alpha)) * ET.tz(l3-l5*np.cos(alpha)))
# robot = rtb.Robot([T_w_motor, T_motor_rodCOM], name="qube robot")

# joint1 = rtb.RevoluteDH(d=0, a=0, alpha=0, offset=0, qlim=(-0.01, 0.01))
joint1 = rtb.RevoluteDH(d=l3, alpha=np.pi/2, a=0, offset=0, qlim=(-0.01, 0.01))
joint2 = rtb.RevoluteDH(d=l4, alpha=np.pi/2, a=0, offset=0, qlim=(-0.01, 0.01))
ee = rtb.RevoluteDH(d=l5, a=0, alpha=0, offset=0, qlim=(-0.01, 0.01))
qube_pendulum_robot = rtb.DHRobot([joint1, joint2, ee], name="Qube+pendulum")

q = np.array([theta, alpha, 0])
print(qube_pendulum_robot.fkine(q))

# qube_pendulum_robot.teach(q)

