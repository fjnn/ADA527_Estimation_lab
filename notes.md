Qube kinematics: 
qube_class.py 
kinematics function 
joint1 = rtb.RevoluteDH(d=self.l3, alpha=np.pi/2, a=0, offset=np.pi, qlim=(-np.pi, np.pi))
joint2 = rtb.RevoluteDH(d=self.l4, alpha=np.pi/2, a=0, offset=-np.pi, qlim=(-np.pi, np.pi))
ee = rtb.RevoluteDH(d=self.l5, a=0, alpha=0, offset=0, qlim=(-0.01, 0.01))
qube_pendulum_robot = rtb.DHRobot([joint1, joint2, ee], name="Qube+pendulum")

TypeError: Slider.__init__() takes 5 positional arguments but 7 were given

changed in peter corke toolbox => https://github.com/petercorke/robotics-toolbox-python/issues/481
if robot.isrevolute(j):
                slider = Slider(
                    ax, "q" + str(j), qlim[0, j], qlim[1, j], valinit= np.degrees(q[j]), valfmt= "% .1f°"
                )
            else:
                slider = Slider(
                    ax, "q" + str(j), qlim[0, j], qlim[1, j], valinit= robot.q[j], valfmt= "% .1f°"
                )

Works!

tested on still image, worked! 


Pixel position: 
red_rectangle_class.py
middle_x = x + w // 2
middle_y = y + h // 2

worked, but got undistort error at the end - m,aybe not a problem??

Implement the Kalman filter 
KF qube using pixel.py 
A = np.array([[1, 0, dt, 0], [0, 1, 0 ,dt], [0, 0, 1, 0], [0, 0, 0, 1]])
B = np.array([[0, 0, 0, 0]]).T
C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
x0 = np.zeros((4, 1), np.float32) # Initial state estimate
y0 = np.zeros((2, 1), np.float32)
Q = np.eye(4) * 0.1  # Process noise covariance
R = np.eye(2) * 0.01  # Measurement noise covariance
P = np.eye(4)  # Initial state covariance

estimated_data.csv created! yey!

PART 1 works! YEYE 


Calibration
1. done
2. skiped
3. record might have to restart qube 
4. changed some file names 