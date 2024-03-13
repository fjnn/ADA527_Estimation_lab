from Classes import Qube
from time import sleep

qube_object = Qube()

for i in range(10):
    qube_object.read_encoders_once()
    sleep(1)

qube_object.close_all()