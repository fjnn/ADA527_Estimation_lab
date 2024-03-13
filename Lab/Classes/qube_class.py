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


class Qube:

    def __init__(self):

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

            # read from Qube (we only need the encoders)
            self.card.task_read(self.task, 1, analog_buffer=None, encoder_buffer=self.encoder_buffer, digital_buffer=None, other_buffer=self.other_buffer)
            
            # Counts to radians
            theta_rad = -2*math.pi/512/4*self.encoder_buffer[0]
            alpha_rad = (2*math.pi/512/4*self.encoder_buffer[1]) % (2 * math.pi) - math.pi

            print("theta: ", degrees(theta_rad), "   alpha: ", degrees(alpha_rad))
        except Exception as e: 
            self.exception_handler()


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

                print("theta: ", degrees(theta_rad), "   alpha: ", degrees(alpha_rad))

            print("Shutting down...")
          
            # self.close_all()
            
            print('Have a nice day!')

            
            return

        except Exception as e: 
            self.exception_handler()