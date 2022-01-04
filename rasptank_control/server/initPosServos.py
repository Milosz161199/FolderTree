#!/usr/bin/python3
# File name   : init
# Description : Control Servos
# Author      : William
# Date        : 2019/02/23
import time
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

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


pwm.set_all_pwm(0, 300)
time.sleep(1)
clean_all()