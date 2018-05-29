#!/usr/bin/env python3

import serial
import time

def read_data(ser,type):
    print('new packet for: {}'.format(type))
    ser.flushInput()
    if type=='pos':
        ser.write(bytearray([0xFF,0x04,0x03,0xfe]))
    elif type=='current':
        ser.write(bytearray([0xFF,0x04,0x06,0xfe]))
    elif type=='torque':
        ser.write(bytearray([0xFF,0x04,0x07,0xfe]))
    else:
        print('invalid input')
    start_timer = time.time()
    ret = None
    while time.time()-start_timer < 0.15 and not ret:
        ret = ser.read()
    if not ret:
        print('didn''t get header in time: {}'.format(time.time()-start_timer))
        return
    header = ord(ret)
    if header != 0xFF:
        print('first byte read did not match header: {}'.format(header))
        return
    ret = None
    while time.time()-start_timer < 0.15 and not ret:
        ret = ser.read()
    if not ret:
        print('didn''t get length in time: {}'.format(time.time()-start_timer))
        return
    length = ord(ret)
    if not length==40:
        print('illogical length byte: {}'.format(length))
        return
    total_read = 2
    data_buffer = [0] * (length-2)
    while time.time()-start_timer < 0.5 and total_read < length :
        ret = ser.read(1)
        if ret:
            data_buffer[total_read - 2] = ord(ret)
            total_read += 1
    if total_read < length:
        print('didn''t receive the entire packet in time: {}'.format(time.time()-start_timer))
        return
    if data_buffer[-1] != 0xFE:
        print('finally data entry was not end byte: {}'.format(data_buffer[-1]))
        return
    data_buffer = data_buffer[0:-2]
    final_joint_pos = [0]*18
    for i in range(18):
        final_joint_pos[i] = (data_buffer[2*i]<<8) + data_buffer[2*i + 1]
    if type=='current':
        final_joint_pos = [fjp / 200.0 for fjp in final_joint_pos]
    print(final_joint_pos)
    print('loop time: {}'.format(time.time()-start_timer))
    return final_joint_pos 

def read_battery_voltage(ser):
    print('new packet to read power level')
    ser.write(bytearray([0xFF,0x04,0x08,0xfe]))
    start_timer = time.time()
    ret = None
    while time.time()-start_timer < 0.15 and not ret:
        ret = ser.read()
    if not ret:
        print('didn''t get header in time: {}'.format(time.time()-start_timer))
        return
    header = ord(ret)
    if header != 0xFF:
        print('first byte read did not match header: {}'.format(header))
        return
    ret = None
    while time.time()-start_timer < 0.15 and not ret:
        ret = ser.read()
    if not ret:
        print('didn''t get length in time: {}'.format(time.time()-start_timer))
        return
    length = ord(ret)
    if not length==6:
        print('illogical length byte: {}'.format(length))
        return
    total_read = 2
    data_buffer = [0] * (length-2)
    while time.time()-start_timer < 0.5 and total_read < length :
        ret = ser.read(1)
        if ret:
            data_buffer[total_read - 2] = ord(ret)
            total_read += 1
    if total_read < length:
        print('didn''t receive the entire packet in time: {}'.format(time.time()-start_timer))
        return
    if data_buffer[-1] != 0xFE:
        print('finally data entry was not end byte: {}'.format(data_buffer[-1]))
        return
    data_buffer = data_buffer[0:-2]
    final_joint_pos = [0]*18
    voltage = ((data_buffer[0]<<8) + data_buffer[1])*0.0124
    print(voltage)
    print('loop time: {}'.format(time.time()-start_timer))
    return voltage 

if __name__ == "__main__":
    print('imports done')
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0.001)
    print('connection established')

    while True:
        read_data(ser,'pos')
        # read_data(ser,'current')
        # read_data(ser,'torque')
        # read_battery_voltage(ser)