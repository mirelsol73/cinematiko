#!/usr/bin/python
# -*- coding: utf-8 -*-

# See : http://www.gigamegablog.com/2012/09/09/beaglebone-coding-101-spi-output/
from spi import SPI

my_spi = SPI(2, 0) # Bus 2, device 0
my_spi.writebytes([0x01])
print("Data sent!")
