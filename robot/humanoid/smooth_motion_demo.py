#!/usr/bin/env python3

import serial
import time

print('imports done')

ser = serial.Serial('/dev/ttyUSB0',115200)
print('connection established')
# ff:05:03:01:fe
for i in range(150,0,-5):
    ser.write(bytearray([0xFF,0x07,0x02,0x02,0x02,0x10+i,0xfe]))
    ret = ser.read(ser.in_waiting)
    print('iteration {}, returned: {}'.format(i,ret))
    # time.sleep(0.1)
    # time.sleep(0.01)
    time.sleep(0.025)

for i in range(150):
    ser.write(bytearray([0xFF,0x07,0x02,0x02,0x02,0x10+i,0xfe]))
    ret = ser.read(ser.in_waiting)
    print('iteration {}, returned: {}'.format(i,ret))
    # time.sleep(0.1)
    # time.sleep(0.01)
    time.sleep(0.001)