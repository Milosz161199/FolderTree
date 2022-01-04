#!/usr/bin/env python3
# File name   : servo.py
# Description : Control Motor
# Product     : RaspTank  
# Website     : www.adeept.com
# E-mail      : support@adeept.com
# Author      : William
# Date        : 2018/12/27
from __future__ import division
import time
import RPi.GPIO as GPIO
import sys
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

'''
11 -  CAMERA servo
12 - 3 servo
13 - 2 servo
14 - 1 servo
15 - HEAD servo

'''

def initPosAll():
	pwm.set_all_pwm(0, 300)
    
#CAMERA SERVO 11
def ahead():
	pwm.set_pwm(11,0,300)

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
 
initPosAll()
# 0.45 unit eq 1°
#servo 13 max w dół od 300 do 400 ruch o max 100 'kąt około 45stopni'
#servo 13 max w górę od 300 do 100 ruch o max 200 'kąt około 90stopni'
servo_13_setting_val = 300;
servo_13_after_move = 0;
servo_13_step_to = 400;
servo_13_max = 400; 
servo_13_min = 100; 

'''
for i in range(0, 200):
    servo_13_setting_val = servo_13_setting_val - i;
    pwm.set_pwm(13, 0, servo_13_setting_val)
    time.sleep(0.01)
print(servo_13_setting_val)
'''
while servo_13_step_to != servo_13_setting_val:
    servo_13_setting_val += 1;
    pwm.set_pwm(13, 0, servo_13_setting_val)
    time.sleep(0.01)
print(servo_13_setting_val) 

servo_13_step_to = 100;
while servo_13_step_to != servo_13_setting_val:
    servo_13_setting_val -= 1;
    pwm.set_pwm(13, 0, servo_13_setting_val)
    time.sleep(0.01)
print(servo_13_setting_val) 

time.sleep(1)
clean_all()