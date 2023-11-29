#!/usr/bin/env python3
# pylint: skip-file

from read_from_bolide import BolideReader
import serial
import time
import struct

print('imports done')

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=.01)
time.sleep(2)
print('connection established')

CMD_version_read = 0x00
CMD_init_motor = 0x01
CMD_set_motor = 0x02
CMD_capture_pos = 0x03
CMD_relax_motor = 0x04
CMD_SN_read = 0x05
CMD_capture_current = 0x06
CMD_capture_torque = 0x07
CMD_capture_battery = 0x08
CMD_SEQ_load_PoseCnt = 0x10
CMD_SEQ_load_Pose = 0x11
CMD_SEQ_load_SEQCnt = 0x12
CMD_SEQ_load_SEQ = 0x13
CMD_SEQ_loop_load_PoseCnt = 0x14
CMD_SEQ_relax = 0x20
CMD_SEQ_halt = 0x30

NUM_MOTORS = 18
##why are we writing bytearrays and using bytes in the first place?
# for i in range(0x00,0xff,8):
#     ser.write(bytearray([0xFF,0x07,0x02,0x02,0x02,0x00+i,0xfe]))
#     ret = ser.read(ser.in_waiting)
#     print('iteration {}, returned: {}'.format(i,[ord(r) for r in ret]))
#     # time.sleep(0.1)
#     # time.sleep(0.01)
#     time.sleep(0.1)

# using time:
# time.sleep(2)
# print('starting first motion')
# ser.write(bytearray([0xFF,0x08,0x02,0x02,0x02,0xf0,0xff,0xfe]))
# ret = ser.read(ser.in_waiting)
# print('returned: {}'.format(ret))
# time.sleep(2)
# print('starting second motion')
# ser.write(bytearray([0xFF,0x08,0x02,0x02,0x02,0x00,0xff,0xfe]))
# ret = ser.read(ser.in_waiting)
# print('returned: {}'.format(ret))


br = BolideReader(ser)


def send_packet(ser, command):
    to_send = bytearray([0xff, len(command)+3]+command+[0xfe])
    print('sending: {}'.format([hex(s) for s in to_send]))
    ser.write(to_send)
    # feedback = br.read_feedback()
    # print(feedback)
    time.sleep(.1)
    print('received: {}'.format([ord(r) for r in ser.read_all()]))
    # ret = ser.read(ser.in_waiting)
    # print('sent: {}'.format(command))
    # if ret:
    #     print('received: {}'.format([ord(c) for c in ret]))

# for i in range(0xff,0x00,-5):
#     send_packet(ser, [0x02,0x02,0x02,0x00+i])

# for i in range(0x00,0xff,8):
#     send_packet(ser,[0x02,0x02,0x02,0x00+i,0xfe])


def upload_pose(ser, id, pose):
    command = [None] * (NUM_MOTORS*2 + 2)
    command[0] = CMD_SEQ_load_Pose
    command[1] = id
    for idx, motor in enumerate(pose):
        bb, lb = struct.pack('>H', motor)
        bb = (ord(bb))
        lb = (ord(lb))
        command[idx*2+2] = bb
        command[idx*2+3] = lb
    send_packet(ser, command)


motors = {'L-shoulder-flex/exten': 1, 'L-shoulder-abduct': 2, 'L-med-rot': 4,
          'L-elbow-flex/exten': 11}

ser.flush()

send_packet(ser, [CMD_init_motor, NUM_MOTORS])
send_packet(ser, [CMD_SEQ_load_PoseCnt, 2])

pose1 = [775, 788, 268, 65535, 504, 65535, 135, 672, 65535,
         65535, 65535, 764, 65535, 65535, 65535, 65535, 65535, 65535]
pose2 = [775, 543, 269, 65535, 503, 65535, 135, 671, 65535,
         65535, 65535, 567, 65535, 65535, 65535, 65535, 65535, 65535]
upload_pose(ser, 0, pose1)
upload_pose(ser, 1, pose2)

send_packet(ser, [CMD_SEQ_load_SEQCnt, 3])
send_packet(ser, [CMD_SEQ_load_SEQ, 0, 0x0, 0x0f])
send_packet(ser, [CMD_SEQ_load_SEQ, 1, 0x0d, 0xff])
send_packet(ser, [CMD_SEQ_load_SEQ, 0, 0x0d, 0xff])

time.sleep(5)
# # ser.write(bytearray([0xFF,0x04,CMD_relax_motor,0xfe]))
send_packet(ser, [CMD_SEQ_relax])
