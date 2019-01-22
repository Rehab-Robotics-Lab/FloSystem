#!/usr/bin/env python3

import serial
import time
import datetime
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
from matplotlib import cm
import time

logging_level = 2

def log(level, message):
    ''' Print the logged info if the current level is high enough. 

    Args:
        level: The level of this message: 0 - very unimportant, 1 - unimportant, 
               2 - marginally important, 3 - important, 4 - very important, 
               5 - critically important. 
        message: The message to print
    '''
    if level >= logging_level:
        print('[{}] {}'.format(datetime.datetime.now(), message))

class BolideReader:
    commands = {'pos':0x03,'current':0x06,'torque':0x07}

    feedback = {'error':0x00,'keep_going':0x01,'done':0x02}
    
    motors = {'L-shoulder-flex/exten':1, 'L-shoulder-abduct':2, 'L-med-rot':4, 
            'L-elbow-flex/exten':11}

    def __init__(self, ser):
        ''' create an object to read data from the bolide robot

        Args:
            ser: The serial object attached to the robot
        '''
        self.ser = ser
        self.servo_vals = np.zeros([0,len(self.motors)])
        self.pos_offsets = np.zeros(len(self.motors),dtype=np.int64)
        self.times = np.zeros(0)
        self.start_time = time.time()
        self.current_time = 0 # elapsed time from start

    def read_data(self, target, tries=5):
        '''Read data from the bolide robot. Check the header, length, and end. 
        Read either position, current, or torque

        Args:
            target: the the targeted thing to read. Options are `pos`, 
                    `current`, `torque`
            tries:  The number of serial read attempts before giving up. 
        '''
        self.ser.flushInput()
        self.ser.write(bytearray([0xFF,0x04,self.commands[target],0xfe]))
        ret = ''
        while len(ret) < 1 and tries > 0:
            ret = self.ser.read(40)
            tries -= 1
        if len(ret)>0:
            header = ord(ret[0])
        else:
            log(3,'not enough data returned after tries')
            return
        if header != 0xFF:
            log(3,'first byte read did not match header: {}'.format(header))
            return
        else:
            good_header = True
        while len(ret) < 40 and tries > 0:
            ret = ret + self.ser.read(40-len(ret))
            tries -= 1
        
        if len(ret) != 40:
            log(3,'wrong length returned, got {}, expected {}'.format(
                len(ret),40))
            return
        
        len_bit = ord(ret[1])
        if len_bit != 40:
            log(3,'wrong length bit sent')
            return
        command = ord(ret[2])
        if not command==self.commands[target]:
            log(3,'incorrect command type: {}'.format(command))
            return
        end = ord(ret[-1])
        if end!=0xFE:
            log(3,'bad end bit')
            return
        data = ret[3:-1]
        final_joint_pos = [0]*18
        for i in range(18):
            final_joint_pos[i] = (ord(data[2*i])<<8) + ord(data[2*i + 1])
        if target=='current':
            final_joint_pos = [fjp / 200.0 for fjp in final_joint_pos]
        # print('{}:{}'.format(com,final_joint_pos))
        return final_joint_pos 

    def read_feedback(self, tries=5):
        '''Read data from the bolide robot. Check the header, length, and end. 
        Read either position, current, or torque

        Args:
            target: the the targeted thing to read. Options are `pos`, 
                    `current`, `torque`
            tries:  The number of serial read attempts before giving up. 
        '''
        ret = ''
        print('starting to look for feedback')
        while len(ret) < 3 and tries > 0:
            ret = self.ser.read(1)
            tries -= 1

        if len(ret)>0:
            print([ord(r) for r in ret])
            header = ord(ret[0])
        else:
            log(3,'not enough data returned after tries')
            return
        
        if header != 0xFF:
            log(3,'first byte read did not match header: {}'.format(header))
            return
        else:
            good_header = True

        while len(ret) < ord(ret[1]) and tries > 0:
            print('starting try')
            ret = ret + self.ser.read(4-len(ret))
            print('done try with: {}'.format(ret))
            tries -= 1

        len_bit = ord(ret[1])
        if len_bit != len(ret):
            log(3,'wrong length bit sent')
            return

        feedback = ord(ret[2])

        end = ord(ret[-1])

        if end!=0xFE:
            log(3,'bad end bit')
            return
        
        return [key for key, value in self.feedback.items() if value == feedback] 


    def read_battery_voltage(self):
        ''' Read battery voltage, not ready for usage
        '''
        self.ser.flushInput()
        self.ser.write(bytearray([0xFF,0x04,0x08,0xfe]))
        ret = self.ser.read(6)
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

    def setup_pos_plots(self):
        '''Set up the pyqt graph to visualize positions'''
        colormap = cm.get_cmap("viridis")  # cm.get_cmap("CMRmap")
        colormap._init()
        # Convert matplotlib colormap from 0-1 to 0 -255 for Qt
        color_lut = (colormap._lut * 255).view(np.ndarray)  

        self.pos_win = pg.GraphicsWindow()
        self.pos_win.setWindowTitle('Robot Position')

        p2 = self.pos_win.addPlot()
        p2.setDownsampling(mode='peak')
        p2.setClipToView(True)
        p2.addLegend()
        self.curves = [p2.plot(pen=pg.mkPen(
            color_lut[idx*color_lut.shape[0]/len(self.motors)]), 
            name=name) for idx,name in enumerate(self.motors.keys())]

    def collect_pos_data(self, to_store=-1):
        '''Ask the robot for its position and store the result
        
        Args:
            to_store: The number of data points to store. If less than zero, 
                      will store all data points
        '''
        position = self.read_data('pos')
        if position:
            new_vals = np.asarray(position)[self.motors.values()]
            # detect wraparound and unwrap:
            # if self.servo_vals.shape[0] > 0:
            #     end_range = 100
            #     upper = 1023-end_range
            #     prior_vals = self.servo_vals[-1] - self.pos_offsets
            #     wrap_up = np.logical_and(new_vals < end_range, 
            #                              prior_vals > upper)
            #     wrap_down = np.logical_and(new_vals > upper, 
            #                                prior_vals < end_range)
            #     self.pos_offsets += wrap_up * 1032
            #     self.pos_offsets -= wrap_down * 1032
            #     # TODO: Why is this 1032? Is 1032 2*pi or is that some wraparound effect?
            #     print(prior_vals)
            #     new_vals += self.pos_offsets
            self.servo_vals = np.append(self.servo_vals,np.atleast_2d(
                new_vals),axis=0)
            self.current_time = time.time()- self.start_time
            self.times = np.append(self.times, self.current_time)
            if to_store > 0:
                self.times = self.times[-to_store:]
                self.servo_vals = self.servo_vals[-to_store:,:]
            return position
        return False

    def update_pos_plot(self, time_to_plot=15):
        '''updates the position plots

        Args:
            time_to_plot: The amount of time back to plot in seconds
        '''
        if len(self.times)>0:
            plot_min_idx = np.argmax(self.times>self.current_time-15)
            for idx, curve in enumerate(self.curves):
                curve.setData(self.times[plot_min_idx:],
                              self.servo_vals[plot_min_idx:,idx])
            QtGui.QApplication.processEvents()
            self.pos_win.repaint()
    
    def print_pos(self, position):
        toprint = ''
        for motor in self.motors.keys():
            toprint += '{}:{}\t'.format(motor,position[self.motors[motor]])
        print(toprint)
        print(position)

if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0.2)
    print('connection established')

    robot = BolideReader(ser)
    robot.setup_pos_plots()
    while True:
        pos = robot.collect_pos_data()
        if pos:
            robot.print_pos(pos)
        robot.update_pos_plot()
