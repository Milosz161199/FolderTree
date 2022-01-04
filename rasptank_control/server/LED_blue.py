#!/usr/bin/python3
# File name   : LED_Police_RED.py
# Description : WS_2812
# Website     : based on the code from https://github.com/rpi-ws281x/rpi-ws281x-python/blob/master/examples/strandtest.py
# Author      : Miłosz Plutowski
# Date        : 2021/04/10

import LED

led = LED.LED()

try:
    led.colorWipe(0,0,255) #RED
except:
    led.colorWipe(0,0,0) # TURN OFF LEDs
