#!/usr/bin/python3
# File name   : Ultrasonic.py
# Device      : HC-SR04
# Description : Detection distance and tracking with ultrasonic
# Website     : www.gewbot.com
# Author      : Miłosz Plutowski
# Date        : 2021/04/11

# Distance range 2cm - 400cm

import RPi.GPIO as GPIO
import time
import json

# Pin numbers
Tr = 11   # Pin Trig (output)
Ec = 8    # Pin Echo (input)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(Tr, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Ec, GPIO.IN)

# data dictionary
dict_data = {}

'''
    @brief Reading distance
'''
def checkdist():       
    for i in range(5):  # Remove invalid test results.
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Tr, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Ec, GPIO.IN)
        GPIO.output(Tr, GPIO.LOW)
        time.sleep(0.00002)
        GPIO.output(Tr, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(Tr, GPIO.LOW)
            
        while not GPIO.input(Ec):
            pass
        t1 = time.time()
        while GPIO.input(Ec):
            pass
        t2 = time.time()
                
            # S = (T2 - T1)x Vs / 2 
            # S - the distance of the obstacle
            # T2 - the time when the echo is received
            # T1 - the time when the sound wave is emitted
            # Vs - the speed of sound propagation in the air
        pulse_duration = t2 - t1
        dist = pulse_duration * 17150.0 / 1.0
        dist = round(dist, 2)
        if dist >= 5 and dist <= 60:  # 5 consecutive times are invalid data, return the last test data        
            return dist
        else:
            return 12
            
if __name__ == '__main__':
    try:
        distance = checkdist()
        #distance = 0.0
    except:
        distance = 0.0
    dict_data['name'] = 'distance'
    dict_data['value'] = distance 
    dict_data['unit'] = 'cm'
    temp = dict_data
    temp_json = json.dumps(temp)
    print(temp_json)