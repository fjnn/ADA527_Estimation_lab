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
 

def ddt_filter(u, state, A, Ts):
    # d/dt with filtering:
    # y = As*u/(s+A)
    #
    # Z-domain with Tustin transform:
    # y = (2AZ - 2A)/((2+AT)z + (AT-2))
    #
    # Divide through by z to get z^-1 terms the convert to time domain
    # y_k1(AT-2) + y_k(AT+2) = 2Au_k - 2Au_k1
    # y_k = 1/(AT+2) * ( 2Au_k - 2Au_k1 - y_k1(AT-2) )
    #
    # y - output
    # u - input
    # state - previous state returned by this function -- initialize to np.array([0,0], dtype=np.float64) 
    # Ts - sample time in seconds
    # A - filter bandwidth in rad/s
        
    y = 1/(A*Ts+2)*(2*A*u - 2*A*state[0] - state[1]*(A*Ts - 2))
    
    state[0] = u
    state[1] = y  
    
    return y, state


def lp_filter(u, state, A, Ts):
    # y = A*u/(s+A)
    #
    # y - output
    # u - input
    # state - previous state returned by this function -- initialize to np.array([0,0], dtype=np.float64) 
    # Ts - sample time in seconds
    # A - filter bandwidth in rad/s
    
    y = (u*Ts*A + state[0]*Ts*A - state[1]*(Ts*A - 2) ) / (2 + Ts*A)
    
    state[0] = u
    state[1] = y
    
    return y, state

def createSquareWave(squareWaveFreq,squareWaveAmplitude, frequency, timeSamples):
    
    period = 1/squareWaveFreq 
    samplesInPeriod = period*frequency
    cycles = int(np.ceil(timeSamples/samplesInPeriod)) - 1

    OneCycle = np.concatenate((np.full(int(samplesInPeriod/2), -squareWaveAmplitude), np.full(int(samplesInPeriod/2), squareWaveAmplitude)), axis=None)

    OutputWave = OneCycle

    for x in range(cycles):
        OutputWave = np.concatenate((OutputWave, OneCycle), axis=None)

    return OutputWave

    

def Balance_controller():

    run_time = 5 # seconds

    print('PD Controller Starting... will run for {} seconds'.format(run_time))
    #print('time    theta rad      theta dot      voltage in     voltage out       Error')
    
    # Open the Qube 3
    card = HIL("qube_servo3_usb", "0")
    task = None
    
    #If you want to change any board-specific options, it can be done here
    #card.set_card_specific_options("deadband_compensation=0.65, pwm_en=0, enc0_velocity=3q.0, enc1_velocity=3.0", 4)
    card.set_card_specific_options("deadband_compensation=0.65", MAX_STRING_LENGTH)
    
    # Create a list of channels to access
    analog_channels_read = np.array([0], dtype=np.uint32)
    encoder_channels_read = np.array([0, 1], dtype=np.uint32)
    digital_channels_read = np.array([0, 1, 2], dtype=np.uint32)
    other_channels_read = np.array([14000, 14001], dtype=np.uint32)
    
    analog_channels_write = np.array([0], dtype=np.uint32)
    digital_channels_write = np.array([0], dtype=np.uint32)
    other_channels_write = np.array([11000, 11001, 11002], dtype=np.uint32)

    # Create read buffers to receive data
    analog_buffer = np.zeros(len(analog_channels_read), dtype=np.float64)
    encoder_buffer = np.zeros(len(encoder_channels_read), dtype=np.int32)
    digital_buffer = np.zeros(len(digital_channels_read), dtype=np.int8)
    other_buffer = np.zeros(len(other_channels_read), dtype=np.float64)
    

    try:
    
        # reset both encoders to values of 0
        card.set_encoder_counts(encoder_channels_read, len(encoder_channels_read), np.array([0, 0], dtype=np.int32))
        
        # set LED's [Red, Green, Blue]
        card.write_other(other_channels_write, len(other_channels_write), np.array([1,1,0], dtype=np.float64))  
        
        # set the initial motor voltage to zero prior to enabling the amplifier
        card.write_analog(analog_channels_write, len(analog_channels_write), np.array([0], dtype=np.float64))
        
        # enable amplifier
        card.write_digital(digital_channels_write, len(digital_channels_write), np.array([1], dtype=np.int8))
        

        #        ref ┌────┐ e ┌───┐   ┌────┐ V ┌─────┐ Vsat ┌──────────────┐   ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
        # Square ───►│+ - ├──►│ P ├──►│+ - ├──►│ _/‾ ├─────►│ Write Analog ├──►│ Qube-Servo 3 ├──►│ Read Encoder ├──►│ Count to Rad ├──┐
        #  Wave      └────┘   └───┘   └────┘   └─────┘      └──────────────┘   └──────────────┘   └──────────────┘   └──────────────┘  │
        #               ▲                ▲                                                     ┌─────────┐                             │
        #               │                │                                     ┌───┐           │  100s   │                             │
        #               │                └─────────────────────────────────────┤ D │◄──────────┤ ─────── │◄────────────────────────────┤
        #               │                                                      └───┘           │ s + 100 │                             │
        #               │                                                                      └─────────┘                             │
        #               └──────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

        
        #initialize states for the derivative term
        state_theta_dot = np.array([0,0], dtype=np.float64) 
        state_alpha_dot = np.array([0,0], dtype=np.float64) 
        
        
        # Buffer for any hiccups in Windows timing
        samples_in_buffer = 1000 
        
        # Control loop frequency
        frequency = 1000 # Hz
        samples = 2**32-1 # Run indefinitely
        
        # Create a task for timebase reads
        task = card.task_create_reader(samples_in_buffer,\
                                    analog_channels_read, len(analog_channels_read),\
                                    encoder_channels_read, len(encoder_channels_read),\
                                    digital_channels_read, len(digital_channels_read),\
                                    other_channels_read, len(other_channels_read))
                                
        # Start timing loop
        card.task_start(task, 0, frequency, samples)   

        timeSamples = run_time*frequency

        squareWaveFreq = 0.4
        squareWaveAmplitude = 0.5
        ref = createSquareWave(squareWaveFreq,squareWaveAmplitude, frequency, timeSamples)

        ref = np.zeros(timeSamples)

        # Start control loop
        for index in range(timeSamples):
                
            # read from Qube (we only need the encoders)
            card.task_read(task, 1, analog_buffer, encoder_buffer, digital_buffer, other_buffer)
            
            # Counts to radians
            theta_rad = -2*math.pi/512/4*encoder_buffer[0]
            alpha_rad = (2*math.pi/512/4*encoder_buffer[1]) % (2 * math.pi) - math.pi


            # Calculate angular velocities with filter of 50 rad
            theta_dot, state_theta_dot = ddt_filter(theta_rad, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha_rad, state_alpha_dot, 50, 1/frequency)
            #theta_dot, state_theta_dot = lp_filter(theta_rad, state_theta_dot, 100, 1/frequency)
            #alpha_dot, state_alpha_dot = lp_filter(alpha_rad, state_alpha_dot, 100, 1/frequency)

            #print(" theta_dot:", theta_dot)
            #print(" alpha_dot:", alpha_dot)
            # print("theta: ", degrees(theta_rad), "   alpha: ", degrees(alpha_rad))
            # print("theta_dot: ", degrees(theta_dot), "   alpha_dof: ", degrees(alpha_dot))
            
            # Calculate control gains
            ArmAngleProportionalGain = -2
            ArmAngleDerivativeGain = -2
            PendulumAngleProportionalGain = 30
            PendulumAngleDerivativeGain = 2.5

            threshold = 0.17
            
            #print("theta: ",theta_rad)
            #print("alpha: ", alpha_rad)

            if (abs(alpha_rad) >= threshold):
                V = 0
            else:    
                e = ref[index] - theta_rad
                V = (e*ArmAngleProportionalGain) - (theta_dot*ArmAngleDerivativeGain) - (alpha_rad*PendulumAngleProportionalGain) - (alpha_dot*PendulumAngleDerivativeGain) 

            # Voltage saturation to +/-10V
            Vsat = max(min(-V, 10), -10)
            #print("V: ",Vsat)

            #if (index % 50 == 0):
            #    print(index/frequency, "\t%.3f" % theta_rad, "\t\t%.3f" % theta_dot, "\t\t%.2f" % ref[index], "\t\t%.3f" % V, "\t\t%.3f" % e)
                        
            
            # Write value to motor
            card.write_analog(analog_channels_write, len(analog_channels_write), np.array([Vsat], dtype=np.float64))
            card.write_other(other_channels_write, len(other_channels_write), np.array([0,1,0], dtype=np.float64))  
            
        
        
        print("Shutting down...")
    
        # Set motor voltage
        card.write_analog(analog_channels_write, len(analog_channels_write), np.array([0], dtype=np.float64))
            
        # Set LED's
        card.write_other(other_channels_write, len(other_channels_write), np.array([1,0,0], dtype=np.float64))  
        
        # Disable amplifier
        card.write_digital(digital_channels_write, len(digital_channels_write), np.array([0], dtype=np.int8))
        
        # Stop then destroy task
        card.task_stop(task)
        card.task_delete(task)  

        # Close HIL device
        card.close()
        
        print('Have a nice day!')

        
        return

    except Exception as e: 
        
        traceback.print_exc()
           
        # Something went wrong. Try to shutdown cleanly.    
        if (task):
            print('Stopping task')
        
            card.task_stop(task)
            card.task_delete(task)    
            
            
        print('Closing card')
        card.close()

    
Balance_controller()