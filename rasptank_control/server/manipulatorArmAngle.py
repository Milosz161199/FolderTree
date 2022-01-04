#!/usr/bin/python3
# File name   : manipulatorArmAngle.py
# Description : Control Servo Drive
# Author      : Milosz Plutowski
# Date        : 2021/04/21

import time
import sys
import select
import math
import move
from numpy import *
import RPi.GPIO as GPIO
import Adafruit_PCA9685

import adafruit_motor.servo


from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

# Initialization PWM channels
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)


# SERVO NUMBERs
# Serwo PWM 11 -> camera_rotation
# Serwo PWM 12 -> theta_2
# Serwo PWM 13 -> theta_3
# Serwo PWM 14 -> theta_4
# Serwo PWM 15 -> grab

''' 
    @brief clean/stop all servomotors (turn off all PWM channels)
'''
def clean_all():
	pwm.set_pwm(0, 0, 0)
	pwm.set_pwm(1, 0, 0)
	pwm.set_pwm(2, 0, 0)
	pwm.set_pwm(3, 0, 0)
	pwm.set_pwm(4, 0, 0)
	pwm.set_pwm(5, 0, 0)
	pwm.set_pwm(6, 0, 0)
	pwm.set_pwm(7, 0, 0)
	pwm.set_pwm(8, 0, 0)
	pwm.set_pwm(9, 0, 0)
	pwm.set_pwm(10, 0, 0)
	pwm.set_pwm(11, 0, 0)
	pwm.set_pwm(12, 0, 0)
	pwm.set_pwm(13, 0, 0)
	pwm.set_pwm(14, 0, 0)
	pwm.set_pwm(15, 0, 0)

# Default values
theta_1 = 90
theta_2 = 130
theta_3 = 130
theta_4 = 90
theta_5 = 90
theta_camera = 78

''' 
    @brief basing servomotors
'''
def servoBase():
    kit.servo[11].angle = theta_camera
    kit.servo[12].angle = theta_2
    kit.servo[13].angle = theta_3
    kit.servo[14].angle = theta_4
    kit.servo[15].angle = theta_5
    time.sleep(1)

''' 
    @brief increase angle of servo
    @param servoNumer - servo of number
    @param servoAngle - currnet angle of servo
'''
def servoMoveUp(servoNumer, servoAngle, step):
    if int(servoAngle) < 180:
        servoAngle += step
        if servoAngle > 180:
            servoAngle = 180
        else:
            servoAngle = servoAngle
        kit.servo[int(servoNumer)].angle = int(servoAngle)
    time.sleep(0.25)
    return servoAngle

''' 
    @brief decrease angle of servo
    @param servoNumer - servo of number
    @param servoAngle - currnet angle of servo
'''
def servoMoveDown(servoNumer, servoAngle, step):
    if int(servoAngle) > 0:
        servoAngle -= step
        if servoAngle < 0:
            servoAngle = 0
        else:
            servoAngle = servoAngle
        kit.servo[int(servoNumer)].angle = int(servoAngle)
    time.sleep(0.25)
    return servoAngle

''' 
    @brief increase angle of servo
    @param servoNumer - servo of number
    @param servoAngle - currnet angle of servo
'''
def servoMoveUpCamera(servoNumer, servoAngle, step):
    if int(servoAngle) < 80:
        servoAngle += step
        if servoAngle > 80:
            servoAngle = 80
        else:
            servoAngle = servoAngle
        kit.servo[int(servoNumer)].angle = int(servoAngle)
    time.sleep(0.25)
    return servoAngle

''' 
    @brief decrease angle of servo
    @param servoNumer - servo of number
    @param servoAngle - currnet angle of servo
'''
def servoMoveDownCamera(servoNumer, servoAngle, step):
    if int(servoAngle) > 30:
        servoAngle -= step
        if servoAngle < 30:
            servoAngle = 30
        else:
            servoAngle = servoAngle
        kit.servo[int(servoNumer)].angle = int(servoAngle)
    time.sleep(0.25)
    return servoAngle

def servoGrab():
    
    move.move(50, 'forward', 'no', 0.8)
    time.sleep(0.075)
    move.motorStop()
    
    # open 
    for a in range(90,-1,-30):
        kit.servo[15].angle = a
        time.sleep(0.01)
    kit.servo[15].angle = 0
    
    # move theta 3 140 -> 50
    for a in range(130,40,-5):
        kit.servo[13].angle = a
        kit.servo[12].angle = a
        time.sleep(0.05)
        
    # move theta 2 140 -> 30
    for a in range(40,0,-5):
        kit.servo[12].angle = a
        time.sleep(0.05)
        
    # close 
    for a in range(0,100,10):
        kit.servo[15].angle = a
        time.sleep(0.05)
    
    time.sleep(2)
    
    # move theta 2 30 -> 20
    for a in range(20,50,5):
        kit.servo[12].angle = a
        time.sleep(0.05)
        
    # move theta 3 50 -> 140
    for a in range(50,130,5):
        kit.servo[13].angle = a
        kit.servo[12].angle = a
        time.sleep(0.05)
        
    move.move(50, 'backward', 'no', 0.8)
    time.sleep(0.075)
    move.motorStop()
    
def servoLose():
    move.move(50, 'forward', 'no', 0.8)
    time.sleep(0.075)
    move.motorStop()

    # move theta 3 140 -> 50
    for a in range(130,60,-5):
        kit.servo[13].angle = a
        kit.servo[12].angle = a
        time.sleep(0.05)
        
    for a in range(60,40,-5):
        kit.servo[13].angle = a
        time.sleep(0.05)
        
    time.sleep(1)
    # open 
    kit.servo[15].angle = 0
    time.sleep(2)
    
    # close 
    for a in range(0,90,5):
        kit.servo[15].angle = a
        time.sleep(0.05)

    for a in range(60,80,5):
        kit.servo[13].angle = a
        time.sleep(0.05)
    
    for a in range(80,130,5):
        kit.servo[13].angle = a
        kit.servo[12].angle = a
        time.sleep(0.05)
    
    time.sleep(1) 
    servoBase() 
    clean_all()
    
    #kit.servo[13].angle = theta_3 # 50
    
    #kit.servo[12].angle = theta_2 # 15
    
    #kit.servo[15].angle = theta_5 # 30
    
    #kit.servo[11].angle = theta_camera
    #kit.servo[12].angle = theta_2
    #kit.servo[13].angle = theta_3
    #kit.servo[14].angle = theta_4
    #kit.servo[15].angle = theta_5
    #time.sleep(1)

def RotateHead():
    for a in range(90,30,-5):
        kit.servo[14].angle = a
        time.sleep(0.05)
    for a in range(30,150,5):
        kit.servo[14].angle = a
        time.sleep(0.05)
    for a in range(150,90,-5):
        kit.servo[14].angle = a
        time.sleep(0.05)
        
if __name__ == '__main__':
    servoBase() 