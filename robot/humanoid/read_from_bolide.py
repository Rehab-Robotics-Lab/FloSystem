#!/usr/bin/env python3

import serial
import time

commands = {'pos':0x03,'current':0x06,'torque':0x07}

motors = {'L-shoulder-flex/exten':1, 'L-shoulder-abduct':2, 'L-med-rot':4, 'L-elbow-flex/exten':11}

def read_data(ser,com):
    # print('new packet for: {}'.format(type))
    ser.flushInput()
    ser.write(bytearray([0xFF,0x04,commands[com],0xfe]))
    ret = ser.read(40)
    if len(ret) != 40:
        # print('wrong length returned, got {}, expected {}'.format(len(ret),40))
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
    ser.flushInput()
    ser.write(bytearray([0xFF,0x04,0x08,0xfe]))
    ret = ser.read(6)
    if len(ret) != 6:
        print('wrong length returned')
        return
    header = ord(ret[0])
    if header != 0xFF:
        print('first byte read did not match header: {}'.format(header))
        return
    len_bit = ord(ret[1])
    if len_bit != 6:
        print('wrong length bit sent')
        return
    command = ord(ret[2])
    if not command==0x08:
        print('incorrect command type: {}'.format(command))
        return
    end = ord(ret[-1])
    if end!=0xFE:
        print('bad end bit')
        return
    data = ret[3:-1]
    final_joint_pos = [0]*18
    voltage = ((ord(data[0])<<8) + ord(data[1]))*0.0124
    print('{}:{}'.format('battery voltage',voltage))
    return voltage 

if __name__ == "__main__":
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtGui
    import numpy as np
    from matplotlib import cm

    colormap = cm.get_cmap("viridis")  # cm.get_cmap("CMRmap")
    colormap._init()
    color_lut = (colormap._lut * 255).view(np.ndarray)  # Convert matplotlib colormap from 0-1 to 0 -255 for Qt


    print('imports done')
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0.2)
    print('connection established')


    win = pg.GraphicsWindow()
    win.setWindowTitle('Robot Position')

    servo_vals = np.zeros([1,len(motors)])
    p2 = win.addPlot()
    p2.setDownsampling(mode='peak')
    p2.setClipToView(True)
    p2.addLegend()
    curves = [p2.plot(pen=pg.mkPen(color_lut[idx*color_lut.shape[0]/len(motors)]), name=name) for idx,name in enumerate(motors.keys())]
    # curve2 = p2.plot()

    while True:
        position = read_data(ser,'pos')
        # bat = read_battery_voltage(ser)
        # current = read_data(ser,'current')
        # torque = read_data(ser,'torque')
        # print('---------------------')
        if position:
            print('pos: {}'.format(position))
            servo_vals = np.append(servo_vals,np.atleast_2d(np.asarray(position)[motors.values()]),axis=0)
            # curve2.setData(servo2)
            for idx, curve in enumerate(curves):
                curve.setData(servo_vals[-500:,idx])
            QtGui.QApplication.processEvents()
            win.repaint()
# if __name__ == "__main__":
#     ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0.2)
#     while True:  
#         position = read_data(ser,'pos')