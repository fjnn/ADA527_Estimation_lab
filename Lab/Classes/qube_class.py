from quanser.hardware import HIL, Clock, DigitalState, MAX_STRING_LENGTH
from quanser.common import GenericError


# https://docs.quanser.com/quarc/documentation/python/index.html

import time
import math
import os
import sys
import traceback

import numpy as np
from math import *

from spatialmath import *
import spatialmath.base as base
import roboticstoolbox as rtb
from roboticstoolbox import ET, Link
from math import radians as d2r


class Qube:

    def __init__(self):

        # Distances and lengths
        # Define the transformations for qube and camera
        self.l1 = 350 # camera-qube distance (x-axis)
        self.l2 = 160 # camera height (z-axis)
        self.l3 = 127 # motor height (z-axis)
        self.l4 = 90 # pendulum offset (x-axis)
        self.l5 = 65 # half of rod lenght (-z-axis(!))

        
        R_q_world = base.rotz(np.pi/2)

        # Motor
        # Resistance
        self.Rm = 7.5
        # Current-torque (N-m/A)
        self.kt = 0.042
        # Back-emf constant (V-s/rad)
        self.km = 0.042

        # Rotary Arm
        # Mass (kg)
        self.mr = 0.095
        # Total length (m)
        self.r = 0.085
        # Moment of inertia about pivot (kg-m^2)
        self.Jr = self.mr*(self.r**2)/3
        # Equivalent Viscous Damping Coefficient (N-m-s/rad)
        self.br = 1e-3 # damping tuned heuristically to match QUBE-Sero 2 response

        # Pendulum Link
        # Mass (kg)
        self.mp = 0.024
        # Total length (m)
        self.Lp = 0.129
        # Pendulum center of mass (m)
        self.l = self.Lp/2
        # Moment of inertia about pivot (kg-m^2)
        self.Jp = self.mp*(self.Lp**2)/3
        # Equivalent Viscous Damping Coefficient (N-m-s/rad)
        self.bp = 5e-5 # damping tuned heuristically to match QUBE-Sero 2 response
        # Gravity Constant
        self.g = 9.81

        # Moment of inertia of pendulum about center of mass (kg-m^2)
        self.Jp_cm = self.mp*(self.Lp**2)/12 # used to calculate pendulum energy in swing-up control

        # Open the Qube 3
        self.card = HIL("qube_servo3_usb", "0")
        self.task = None

        self.encoder_channels_read = np.array([0, 1], dtype=np.uint32)
        self.encoder_buffer = np.zeros(len(self.encoder_channels_read), dtype=np.int32)
        self.encoder_buffer = np.zeros(len(self.encoder_channels_read), dtype=np.int32)

        # For LED:
        self.other_channels_read = np.array([14000, 14001], dtype=np.uint32)
        self.other_channels_write = np.array([11000, 11001, 11002], dtype=np.uint32)
        self.other_buffer = np.zeros(len(self.other_channels_read), dtype=np.float64)

        #If you want to change any board-specific options, it can be done here
        #card.set_card_specific_options("deadband_compensation=0.65, pwm_en=0, enc0_velocity=3q.0, enc1_velocity=3.0", 4)
        self.card.set_card_specific_options("deadband_compensation=0.65", MAX_STRING_LENGTH) 

        try:
            # reset both encoders to values of 0
            self.card.set_encoder_counts(self.encoder_channels_read, len(self.encoder_channels_read), np.array([0, 0], dtype=np.int32))
            
            # set LED's [Red, Green, Blue]
            self.card.write_other(self.other_channels_write, len(self.other_channels_write), np.array([1,1,0], dtype=np.float64))  

        except Exception as e: 
            self.exception_handler()

            
        print("initialized")

    
    def exception_handler(self):
        traceback.print_exc()
            
        # Something went wrong. Try to shutdown cleanly.    
        if (self.task):
            print('Stopping task')
        
            self.card.task_stop(self.task)
            self.card.task_delete(self.task)    
            
        print('Closing card')
        self.card.close()


    def close_all(self):  
        try:
            # Set LED's
            self.card.write_other(self.other_channels_write, len(self.other_channels_write), np.array([1,0,0], dtype=np.float64))  
            
            # Stop then destroy task
            self.card.task_stop(self.task)
            self.card.task_delete(self.task)  

            # Close HIL device
            self.card.close()
            
            print('Have a nice day!')
        except Exception as e: 
            self.exception_handler()


    def read_encoders_once(self):
        # print("Started")     

        try:
        
            # Buffer for any hiccups in Windows timing
            samples_in_buffer = 1000 
            
            # Control loop frequency
            frequency = 1000 # Hz
            samples = 2**32-1 # Run indefinitely
            
            # Create a task for timebase reads
            self.task = self.card.task_create_reader(samples_in_buffer,
                                        analog_channels=None, num_analog_channels=0,
                                        encoder_channels=self.encoder_channels_read, num_encoder_channels=len(self.encoder_channels_read),
                                        digital_channels=None, num_digital_channels=0,
                                        other_channels=self.other_channels_read, num_other_channels=len(self.other_channels_read))
                                    
            # print("so far so good")

            # Start timing loop
            self.card.task_start(self.task, 0, frequency, samples)   

            # read from Qube (we only need the encoders)
            self.card.task_read(self.task, 1, analog_buffer=None, encoder_buffer=self.encoder_buffer, digital_buffer=None, other_buffer=self.other_buffer)
            
            # Counts to radians
            theta_rad = -2*math.pi/512/4*self.encoder_buffer[0]
            alpha_rad = (2*math.pi/512/4*self.encoder_buffer[1]) % (2 * math.pi) - math.pi

            # print("theta: ", degrees(theta_rad), "   alpha: ", degrees(alpha_rad))
        except Exception as e: 
            self.exception_handler()
        
        return([theta_rad, alpha_rad])


    def read_encoders(self):

        run_time = 5 # seconds
        print("Started")     

        try:
        
            # Buffer for any hiccups in Windows timing
            samples_in_buffer = 1000 
            
            # Control loop frequency
            frequency = 1000 # Hz
            samples = 2**32-1 # Run indefinitely
            
            # Create a task for timebase reads
            self.task = self.card.task_create_reader(samples_in_buffer,
                                        analog_channels=None, num_analog_channels=0,
                                        encoder_channels=self.encoder_channels_read, num_encoder_channels=len(self.encoder_channels_read),
                                        digital_channels=None, num_digital_channels=0,
                                        other_channels=self.other_channels_read, num_other_channels=len(self.other_channels_read))
                                    
            # print("so far so good")

            # Start timing loop
            self.card.task_start(self.task, 0, frequency, samples)   

            timeSamples = run_time*frequency


            # Start control loop
            for index in range(timeSamples):
                    
                # read from Qube (we only need the encoders)
                self.card.task_read(self.task, 1, analog_buffer=None, encoder_buffer=self.encoder_buffer, digital_buffer=None, other_buffer=self.other_buffer)
                
                # Counts to radians
                theta_rad = -2*math.pi/512/4*self.encoder_buffer[0]
                alpha_rad = (2*math.pi/512/4*self.encoder_buffer[1]) % (2 * math.pi) - math.pi

                # print("theta: ", degrees(theta_rad), "   alpha: ", degrees(alpha_rad))

            print("Shutting down...")
          
            # self.close_all()
            
            print('Have a nice day!')

            
            return

        except Exception as e: 
            self.exception_handler()

    
    def kinematics(self, theta=0, alpha=0):

        ## Qube+Pendulum is like the second robot
        theta = theta
        alpha = alpha
        # T_w_motor = Link(ET.tz(l3) * ET.Rz(theta))
        # T_motor_rodCOM = Link(ET.Rz(0) * ET.tx(l4) * ET.ty(l5*np.sin(alpha)) * ET.tz(l3-l5*np.cos(alpha)))
        # robot = rtb.Robot([T_w_motor, T_motor_rodCOM], name="qube robot")

        # joint1 = rtb.RevoluteDH(d=0, a=0, alpha=0, offset=0, qlim=(-0.01, 0.01))
        joint1 = rtb.RevoluteDH(d=self.l3, alpha=np.pi/2, a=0, offset=np.pi, qlim=(-np.pi, np.pi))
        joint2 = rtb.RevoluteDH(d=self.l4, alpha=np.pi/2, a=0, offset=-np.pi, qlim=(-np.pi, np.pi))
        ee = rtb.RevoluteDH(d=self.l5, a=0, alpha=0, offset=0, qlim=(-0.01, 0.01))
        qube_pendulum_robot = rtb.DHRobot([joint1, joint2, ee], name="Qube+pendulum")

        q = np.array([theta, alpha, 0])
        ee_position = qube_pendulum_robot.fkine(q).t
        # print(qube_pendulum_robot.fkine(q).A())

        R_q_world = base.rotz(np.pi/2)
        ee_coordinates = np.matmul(R_q_world, np.array([ee_position[0], ee_position[1], ee_position[2]]))

        return ee_coordinates

        qube_pendulum_robot.teach(q)
    
    def qube_to_camera(self, ee_world_coordinates):
        '''
        To visualize encoder measurements on the camera, use these coordinates.
        '''
        ## Camera transformation
        # T_w_camera = Link(ET.tz(self.l1) * ET.Rz(np.pi))
        T_w_camera = Link(ET.Ry(-np.pi/2) * ET.Rz(-np.pi/5) * ET.Rx(-np.pi/2)* ET.Ry(-np.pi/2))
        transformed_ee_coordinates = np.matmul(T_w_camera.A().R, ee_world_coordinates)

        return transformed_ee_coordinates

