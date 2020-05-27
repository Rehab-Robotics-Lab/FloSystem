#!/usr/bin/env python

"""Script for testing the serial connection"""

from __future__ import print_function
import sys
import serial

if len(sys.argv) != 2:
    print("python: Usage_serial_test <port name like: /dev/ttyUSB0>")
    sys.exit(1)

SIO = serial.Serial(sys.argv[1], 115200)
SIO.timeout = 250

while True:
    SIO.write("Testing.")
    print(SIO.read(8))
