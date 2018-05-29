#!/usr/bin/env python3

import serial
import time

commands = {'pos':0x03,'current':0x06,'torque':0x07}

def read_data(ser,com):
    # print('new packet for: {}'.format(type))
    ser.flushInput()
    ser.write(bytearray([0xFF,0x04,commands[com],0xfe]))
    ret = ser.read(40)
    if len(ret) != 40:
        print('wrong length returned')
        return
    header = ord(ret[0])
    if header != 0xFF:
        print('first byte read did not match header: {}'.format(header))
        return
    len_bit = ord(ret[1])
    if len_bit != 40:
        print('wrong length bit sent')
        return
    command = ord(ret[2])
    if not command==commands[com]:
        print('incorrect command type: {}'.format(command))
        return
    end = ord(ret[-1])
    if end!=0xFE:
        print('bad end bit')
        return
    data = ret[3:-1]
    final_joint_pos = [0]*18
    for i in range(18):
        final_joint_pos[i] = (ord(data[2*i])<<8) + ord(data[2*i + 1])
    if com=='current':
        final_joint_pos = [fjp / 200.0 for fjp in final_joint_pos]
    print('{}:{}'.format(com,final_joint_pos))
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
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtGui
    import numpy as np
    print('imports done')
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0.2)
    print('connection established')


    win = pg.GraphicsWindow()
    win.setWindowTitle('Robot Position')

    servo2 = np.zeros(0)
    p2 = win.addPlot()
    p2.setDownsampling(mode='peak')
    p2.setClipToView(True)
    curve2 = p2.plot()

    while True:
        position = read_data(ser,'pos')
        if position:
            servo2 = np.append(servo2,position[1])
            curve2.setData(servo2)
            QtGui.QApplication.processEvents()
            win.repaint()
# if __name__ == "__main__":
#     ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0.2)
#     while True:  
#         position = read_data(ser,'pos')