from Classes import Qube
from time import sleep
from math import degrees

# import numpy as np
# from spatialmath import *
# import spatialmath.base as base
# import roboticstoolbox as rtb
# from roboticstoolbox import ET, Link
# from math import radians as d2r


qube_object = Qube()

try:
    while True:
        encoder_readings = qube_object.read_encoders_once()
        com_encoder = qube_object.kinematics(encoder_readings[0], encoder_readings[1])
        com_encoder = qube_object.qube_to_camera(com_encoder)
        print("com_encoder: ", com_encoder, "angles: ", degrees(encoder_readings[0]), degrees(encoder_readings[1]))
        sleep(0.5)
except KeyboardInterrupt:
    print("KeyboardInterrupt received. Exiting...")

    qube_object.close_all()



