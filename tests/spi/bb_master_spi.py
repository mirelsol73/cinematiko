#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import division # Python 3 facility : 3/4 = 0.75 (and not 0 like in Python 2)

# See : http://www.gigamegablog.com/2012/09/09/beaglebone-coding-101-spi-output/
from spi import SPI
from time import sleep

my_spi = SPI(2, 0) # Bus 2, device 0
my_spi.msh = 4000000 # In Hz
#my_spi.msh = 2000000
#my_spi.mode = 3


# Map from analog value to delay for motor when direction is "high"
def map_analog_to_delay_high(x, in_min, in_max, out_min, out_max):
    return round(out_max - (x - in_min) * (abs(out_max - out_min) / abs(in_max - in_min)))
    #print("mapAnalogToDelayHigh : %f" % (out_max - (x - in_min) * MAP_HIGH))
    #return round(out_max - (x - in_min) * MAP_HIGH)

# Map from analog value to delay for motor when direction is "low"
def map_analog_to_delay_low(x, in_min, in_max, out_min, out_max):
    return round((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    #print("mapAnalogToDelayLow : %f" % ((x - in_min) * MAP_LOW + MOTOR_MIN_DELAY))
    #return round((x - in_min) * MAP_LOW + MOTOR_MIN_DELAY)

while True:
    '''
    val = raw_input('[c]ommand or [d]elay : ')
    if val == 'd':
        delay = raw_input('Enter delay : ')
        #delay = 3
        my_spi.writebytes([int(delay) & 0x00FF]) # Low byte
        my_spi.writebytes([int(delay) >> 8]) # High byte
    elif val == 'c':
        my_spi.writebytes([0x99]) # 'c'
        cmd = raw_input('Enter command : ')
        my_spi.writebytes([int(hex(ord(cmd)), 16)])
    '''
    delay = raw_input('Enter delay : ')
    #delay = 3
    my_spi.writebytes([int(delay) & 0x00FF]) # Low byte
    my_spi.writebytes([int(delay) >> 8]) # High byte
    
    #print(map_analog_to_delay_high(900, 0, 1023, 3, 5000))
    sleep(0.01)

print("Data sent!")
