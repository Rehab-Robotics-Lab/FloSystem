#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import time
import datetime
import serial
import numpy as np
from matplotlib import cm

LOGGING_LEVEL = 6


def log(level, message):
    ''' Print the logged info if the current level is high enough.

    Args:
        level: The level of the message: 0 - very unimportant, 1 - unimportant,
               2 - marginally important, 3 - important, 4 - very important,
               5 - critically important.
        message: The message to print
    '''
    if level >= LOGGING_LEVEL:
        print('[{}] {}'.format(datetime.datetime.now(), message))


class BolideReader(object):

    commands = {'pos': 0x03, 'current': 0x06, 'torque': 0x07}

    feedback = {'error': 0x00, 'keep_going': 0x01,
                'done': 0x02, 'relaxed': 0x03}

    motors = {'L-shoulder-flex/exten': 1, 'L-shoulder-abduct': 2,
              'L-med-rot': 4, 'L-elbow-flex/exten': 11,
              'R-shoulder-flex/exten': 0, 'R-shoulder-abduction': 14,
              'R-shoulder-rotation': 3, 'R-elbow-flex/exten': 5}

    def __init__(self, ser_obj):
        ''' create an object to read data from the bolide robot

        Args:
            ser_obj: The serial object attached to the robot
        '''
        self.ser = ser_obj
        self.servo_vals = np.zeros([0, len(self.motors)])
        self.pos_offsets = np.zeros(len(self.motors), dtype=np.int64)
        self.times = np.zeros(0)
        self.start_time = time.time()
        self.current_time = 0  # elapsed time from start

    def read_data(self, target, tries=5):
        '''Read data from the bolide robot. Check the header, length,
        and end. Read either position, current, or torque

        Args:
            target: the the targeted thing to read. Options are `pos`,
                    `current`, `torque`
            tries:  The number of serial read attempts before giving up.
        '''
        # TODO: not sure if flushing the input is a good idea here
        self.ser.flushInput()
        self.ser.write(bytearray([0xFF, 0x04, self.commands[target], 0xfe]))
        ret = ''
        while len(ret) < 1 and tries > 0:
            # TODO should i actually be waiting for 40 bytes each time?
            ret = self.ser.read(40)
            tries -= 1
            log(1, 'len(ret): {} | ret: {}'.format(len(ret), ret))
        if ret:
            header = ord(ret[0])
        else:
            log(3, 'not enough data returned after tries')
            return
        if header != 0xFF:
            log(3, 'first byte read did not match header: {}'.format(header))
            return
        # If we make it to here, we have received a good header

        while len(ret) < 40 and tries > 0:
            ret = ret + self.ser.read(40-len(ret))
            log(1, 'len(ret): {} | ret: {}'.format(len(ret), ret))
            tries -= 1

        if len(ret) != 40:
            log(3, 'wrong length returned, got {}, expected {}'.format(
                len(ret), 40))
            return

        len_bit = ord(ret[1])
        if len_bit != 40:
            log(3, 'wrong length bit sent')
            return
        command = ord(ret[2])
        if not command == self.commands[target]:
            log(3, 'incorrect command type: {}'.format(command))
            return
        end = ord(ret[-1])
        if end != 0xFE:
            log(3, 'bad end bit')
            return
        data = ret[3:-1]
        final_joint_pos = [0]*18
        for i in range(18):
            final_joint_pos[i] = (ord(data[2*i]) << 8) + ord(data[2*i + 1])
        if target == 'current':
            final_joint_pos = [fjp / 200.0 for fjp in final_joint_pos]
        # print('{}:{}'.format(com,final_joint_pos))
        return final_joint_pos

    def read_feedback(self, tries=5):
        '''Read feedback from robot. Check the header, length,
        and end. Expect header, length=0x04, code, tail

        Args:
            tries:  The number of serial read attempts before giving up.
        '''
        ret = ''
        # print('starting to look for feedback')
        while len(ret) < 2 and tries > 0:
            ret = ret+self.ser.read(1)
            tries -= 1

        if ret:
            header = ord(ret[0])
        else:
            return ('local_err',
                    'not enough data returned after tries. Length of data: {}'.format(len(ret)))

        if header != 0xFF:
            return ('local_err', 'first byte read did not match header: {}'.format(header))
        # If we get to here, we have received a good header
        expected_length = ord(ret[1])
        while len(ret) < expected_length and tries > 0:
            ret = ret + self.ser.read(expected_length-len(ret))
            tries -= 1

        if expected_length != len(ret):
            return ('local_err', 'wrong length bit sent')

        feedback = ord(ret[2])

        end = ord(ret[-1])

        if end != 0xFE:
            return ('local_err', 'bad end bit')

        # return [key for key, value in self.feedback.items() if value == feedback]
        return feedback

    # def read_battery_voltage(self):
    #     ''' Read battery voltage, not ready for usage
    #     '''
    #     self.ser.flushInput()
    #     self.ser.write(bytearray([0xFF, 0x04, 0x08, 0xfe]))
    #     ret = self.ser.read(6)
    #     if len(ret) != 6:
    #         # print('wrong length returned')
    #         return
    #     header = ord(ret[0])
    #     if header != 0xFF:
    #         # print('first byte read did not match header: {}'.format(header))
    #         return
    #     len_bit = ord(ret[1])
    #     if len_bit != 6:
    #         # print('wrong length bit sent')
    #         return
    #     command = ord(ret[2])
    #     if not command == 0x08:
    #         # print('incorrect command type: {}'.format(command))
    #         return
    #     end = ord(ret[-1])
    #     if end != 0xFE:
    #         # print('bad end bit')
    #         return
    #     data = ret[3:-1]
    #     final_joint_pos = [0]*18
    #     voltage = ((ord(data[0]) << 8) + ord(data[1]))*0.0124
    #     # print('{}:{}'.format('battery voltage', voltage))
    #     return voltage

    # def setup_pos_plots(self):
    #     '''Set up the pyqt graph to visualize positions'''
    #     import pyqtgraph as pg
    #     colormap = cm.get_cmap("viridis")  # cm.get_cmap("CMRmap")
    #     colormap._init()
    #     # Convert matplotlib colormap from 0-1 to 0 -255 for Qt
    #     color_lut = (colormap._lut * 255).view(np.ndarray)

    #     self.pos_win = pg.GraphicsWindow()
    #     self.pos_win.setWindowTitle('Robot Position')

    #     p2 = self.pos_win.addPlot()
    #     p2.setDownsampling(mode='peak')
    #     p2.setClipToView(True)
    #     p2.addLegend()
    #     self.curves = [p2.plot(pen=pg.mkPen(
    #         color_lut[idx*color_lut.shape[0]/len(self.motors)]),
    #         name=name) for idx, name in enumerate(self.motors.keys())]

    def collect_pos_data(self, to_store=-1):
        '''Ask the robot for its position and store the result

        Args:
            to_store: The number of data points to store. If less than zero,
                      will store all data points
        '''
        position = self.read_data('pos')
        if position:
            new_vals = np.asarray(position)[self.motors.values()]
            self.servo_vals = np.append(self.servo_vals, np.atleast_2d(
                new_vals), axis=0)
            self.current_time = time.time() - self.start_time
            self.times = np.append(self.times, self.current_time)
            if to_store > 0:
                self.times = self.times[-to_store:]
                self.servo_vals = self.servo_vals[-to_store:, :]
            return position
        return False

    # def update_pos_plot(self, time_to_plot=15):
    #     '''updates the position plots

    #     Args:
    #         time_to_plot: The amount of time back to plot in seconds
    #     '''
    #     from pyqtgraph.Qt import QtCore, QtGui
    #     if len(self.times) > 0:
    #         plot_min_idx = np.argmax(self.times > self.current_time-15)
    #         for idx, curve in enumerate(self.curves):
    #             curve.setData(self.times[plot_min_idx:],
    #                           self.servo_vals[plot_min_idx:, idx])
    #         QtGui.QApplication.processEvents()
    #         self.pos_win.repaint()

    # def print_pos(self, position):
    #     toprint = ''
    #     for motor in self.motors.keys():
    #         toprint += '{}:{}\t'.format(motor, position[self.motors[motor]])
    #     print(toprint)
    #     print(position)


# if __name__ == "__main__":
#     ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)
#     # print('connection established')

#     robot = BolideReader(ser)
#     robot.setup_pos_plots()
#     while True:
#         pos = robot.collect_pos_data()
#         if pos:
#             robot.print_pos(pos)
#         robot.update_pos_plot()
