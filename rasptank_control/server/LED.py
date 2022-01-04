#!/usr/bin/python3
# File name   : LED.py
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
        #parser = argparse.ArgumentParser()
        #parser.add_argument('-c', '--clear', action='store_true', help='clear the display on exit')
        #args = parser.parse_args()

        # Create NeoPixel object with appropriate configuration.
        self.strip = Adafruit_NeoPixel(self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL)
        # Intialize the library (must be called once before other functions).
        self.strip.begin()

    # Define functions which animate LEDs in various ways.
    # Funkcja do zmiany kolorów wszystkich diod LED na raz na wybrany kolor
    def colorWipe(self, r, g, b, wait_ms=0):
        """Wipe color across display a pixel at a time."""
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, Color(r, g, b))
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

if __name__ == '__main__':

#deklaracja obiektu clasy LED
    led = LED()


    '''
    try:
        while True:
            led.colorWipe(Color(255,0,0)) #RED
            time.sleep(1);
            led.colorWipe(Color(0,255,0)) #GREEN
            time.sleep(1);
            led.colorWipe(Color(0,0,255)) #BLUE
            time.sleep(1);
    except:
        led.colorWipe(Color(0,0,0)) # TURN OFF LEDs
    '''
    # zapalanie poszczególnych diod LED
    #led.strip.setPixelColor(2, Color(255,255,0))
    #led.strip.show()



    #dic_data = {'number': '', 'R': 0, 'G': 0, 'B': 0}

'''
    file = open('/home/pi/server/Projekt/leds.dat', "r")
    if file.readable():
        tmp = file.read()
        js = json.loads(tmp)
        print(js)
    file.close()

    for i in js:
        led.strip.setPixelColor(i['number'], Color(i['R'],i['G'],i['B']))
        led.strip.show()
'''
