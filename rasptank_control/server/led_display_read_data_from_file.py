#!/usr/bin/python3
# File name   : led_display_read_data_from_file.py
# Description : WS_2812
# Website     : based on the code from https://github.com/rpi-ws281x/rpi-ws281x-python/blob/master/examples/strandtest.py
# Author      : Miłosz Plutowski
# Date        : 2021/04/10

import time
from rpi_ws281x import *
import argparse
import sys
import select
import json
import os

# LED strip configuration:
LED_COUNT      = 12      # Number of LED pixels.
LED_PIN        = 12      # GPIO pin connected to the pixels (18 uses PWM!).
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

class LED:
    def __init__(self):
        self.LED_COUNT      = 12      # Number of LED pixels.
        self.LED_PIN        = 12      # GPIO pin connected to the pixels (18 uses PWM!).
        self.LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
        self.LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
        self.LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
        self.LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
        self.LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

        # Create NeoPixel object with appropriate configuration.
        self.strip = Adafruit_NeoPixel(self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL)
        # Intialize the library (must be called once before other functions).
        self.strip.begin()

# data dictionary
dict_data = {'x':'', 'y':'', 'r':'', 'g':'','b':''}

if __name__ == '__main__':
    #declaration LED
    led = LED()

    file = open("/home/pi/server/appServer/led_display_test_file.json", "r")
    if file.readable():  
        result = json.loads(file.read())
        #print(result)
    file.close()

    for x in result:
        dict_data['x'] = x[0]
        dict_data['y'] = x[1]
        dict_data['r'] = x[2]
        dict_data['g'] = x[3]
        dict_data['b'] = x[4]
        print(dict_data)
        print(x[0] + x[1]*6)
        led.strip.setPixelColor(x[0] + x[1]*6, Color(x[2],x[3],x[4]))
        led.strip.show()