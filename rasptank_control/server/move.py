#!/usr/bin/env python3
# File name   : move.py
# Description : Control Motor Drive
# Author      : Milosz Plutowski
# Date        : 2021/04/21

import time
import RPi.GPIO as GPIO
from threading import Timer

'''
motor_EN_A: Pin7 on HAT 
motor_EN_B: Pin11 on HAT
motor_A: Pin8, Pin10 on HAT
motor_B: Pin13, Pin12 on HAT
'''

Motor_A_EN = 4
Motor_B_EN = 17

Motor_A_Pin1 = 15
Motor_A_Pin2 = 14
Motor_B_Pin1 = 18
Motor_B_Pin2 = 27

Dir_forward = 0
Dir_backward = 1

left_forward = 0
left_backward = 1

right_forward = 0
right_backward = 1

pwn_A = 0
pwm_B = 0

''' 
    @brief Motors stop
'''


def motorStop():
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_B_Pin1, GPIO.LOW)
    GPIO.output(Motor_B_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW)
    GPIO.output(Motor_B_EN, GPIO.LOW)


''' 
    @brief Motors initialization
'''


def setup():
    global pwm_A, pwm_B
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_B_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    GPIO.setup(Motor_B_Pin1, GPIO.OUT)
    GPIO.setup(Motor_B_Pin2, GPIO.OUT)

    motorStop()
    try:
        pwm_A = GPIO.PWM(Motor_A_EN, 1000)
        pwm_B = GPIO.PWM(Motor_B_EN, 1000)
    except:
        pass


''' 
    @brief Motor 2 (left) positive and negative rotation
    @param status - if activated (true/false)
    @param direction - direction of motor rotation (forward/backward/no)
    @param speed - value of motor speed (PWM)
'''


def motor_left(status, direction, speed):  # Motor 2 positive and negative rotation
    if status == 0:  # stop
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)
    else:
        if direction == Dir_backward:
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
            pwm_B.start(0)
            pwm_B.ChangeDutyCycle(speed)
        elif direction == Dir_forward:
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
            pwm_B.start(0)
            pwm_B.ChangeDutyCycle(speed)


''' 
    @brief Motor 1 (right) positive and negative rotation
    @param status - if activated (true/false)
    @param direction - direction of motor rotation (forward/backward/no)
    @param speed - value of motor speed (PWM)
    @return direction - 
'''


def motor_right(status, direction, speed):  # Motor 1 positive and negative rotation
    if status == 0:  # stop
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
    else:
        if direction == Dir_forward:  #
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            pwm_A.start(0)
            pwm_A.ChangeDutyCycle(speed)
        elif direction == Dir_backward:
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            pwm_A.start(0)
            pwm_A.ChangeDutyCycle(speed)
    return direction


''' 
    @brief main function for control motors
    @param speed - value of motor speed (PWM)
    @param direction - direction of motor rotation (forward/backward/no)
    @param turn - turn robot (right/left)
    @param radius - value of radius to calculate speed
'''


def move(speed, direction, turn, radius=0.6):  # 0 < radius <= 1  
    speed_div = 1.3
    if direction == 'forward':
        if turn == 'right':
            motor_left(1, left_forward, int(speed * radius))
            motor_right(0, right_backward, speed / speed_div)
        elif turn == 'left':
            motor_left(0, left_backward, speed / speed_div)
            motor_right(1, right_forward, int(speed * radius))
        else:
            motor_left(1, left_forward, speed)
            motor_right(1, right_forward, speed)
    elif direction == 'backward':
        if turn == 'right':
            motor_left(1, left_backward, int(speed * radius))
            motor_right(0, right_forward, speed / speed_div)
        elif turn == 'left':
            motor_left(0, left_forward, speed / speed_div)
            motor_right(1, right_backward, int(speed * radius))
        else:
            motor_left(1, left_backward, speed)
            motor_right(1, right_backward, speed)
    elif direction == 'no':
        if turn == 'right':
            motor_left(1, left_forward, speed)
            motor_right(1, right_backward, speed)
        elif turn == 'left':
            motor_left(1, left_backward, speed)
            motor_right(1, right_forward, speed)
        else:
            motorStop()
    else:
        pass


def moveNEW(speed, direction, turn, radius=0.6):  # 0 < radius <= 1  
    speed_div = 1.3
    for i in range(0, speed, 10):
        if direction == 'forward':
            if turn == 'right':
                motor_left(1, left_forward, int(i * radius))
                motor_right(0, right_backward, int(i / speed_div))
            elif turn == 'left':
                motor_left(0, left_backward, int(i / speed_div))
                motor_right(1, right_forward, int(i * radius))
            else:
                motor_left(1, left_forward, i)
                motor_right(1, right_forward, i)
        elif direction == 'backward':
            if turn == 'right':
                motor_left(1, left_backward, int(i * radius))
                motor_right(0, right_forward, int(i / speed_div))
            elif turn == 'left':
                motor_left(0, left_forward, int(i / speed_div))
                motor_right(1, right_backward, int(i * radius))
            else:
                motor_left(1, left_backward, i)
                motor_right(1, right_backward, i)
        elif direction == 'no':
            if turn == 'right':
                motor_left(1, left_forward, i)
                motor_right(1, right_backward, i)
            elif turn == 'left':
                motor_left(1, left_backward, i)
                motor_right(1, right_forward, i)
            else:
                motorStop()
        else:
            pass
        time.sleep(0.05)


''' 
    @brief Motors stop and clear GPIO
'''


def destroy():
    motorStop()
    GPIO.cleanup()


if __name__ == '__main__':
    setup()
