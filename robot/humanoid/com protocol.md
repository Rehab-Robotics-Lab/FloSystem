Packet to robot via serial:

Serial running at: 115200


    Header (1 byte) | Packet Length (1 byte) |       CMD (1 byte)         | data | tail
          0xFF      |       (ex. 0x07)       | see Y-01_Mask_Definition.h |      | 0xFE


First Packet to init motors:
ff:05:01:12:fe
packet header, packet length of 5 bytes, init motors, number of servos in the pose (why 12?), packet tail

Control Motors:
ff:07:02:01:02:9d:fe
packet header, packet length of 7 bytes, set motor position, motor ID, motor position 1, motor position 2 number of servos in the pose (why 12?), packet tail
- final motor position = motor position 1 << 8 + motor position 2
- expect a response that is a copy ff:07:02:01:02:9d:fe

Get Serial number:
ff:04:05:fe
packet header, packet length of 4 bytes, request serial number, packet tail

Get Motor Position:
ff:05:03:01:fe
packet header, packet length of 5 bytes, request motor position, motor id, packet tail
the motor ID does not seem important
Returns: wtf?
        ff:29:03:01:...:fe
        header, length (41), request motor position, motor id, 19x2 pos(n)<<8+pos(n) for all poses, tail

Notes:
Currently there is no speed control on the motors. To edit:
 - Change line 738 of Bolide_Y-01... to pass a value other than 0x00 for the time to spend
    - will need to read in a time over the network

Find the serial port attached on linux: `dmesg | grep tty`
You may need to make it read writeable: sudo chmod 666 /dev/ttyUSB0

ex:
```python
import serial
# ser = serial.Serial('COM4',115200) Windows
ser = serial.Serial('/dev/ttyUSB0',115200)
ser.write(bytearray([0xFF,0x07,0x02,0x01,0x02,0x9d,0xfe]))
ser.read(ser.in_waiting)
ser.write(bytearray([0xFF,0x07,0x02,0x01,0x01,0x9d,0xfe]))
ser.read(ser.in_waiting)
ser.write(bytearray([0xFF,0x05,0x03,0x01,0xfe]))
ser.read(ser.in_waiting)
ser.write(bytearray([0xFF,0x05,0x04,0x01,0xfe]))
ser.read(ser.in_waiting)
ser.close()
```

A demo of smoothish motion:

```python
import serial
import time

for i in range(100):
    ser.write(bytearray([0xFF,0x07,0x02,0x02,0x02,0x10+i,0xfe]))
    time.sleep(0.5)
```
Motion is ok, but a bit jerky. I think it would make sense to play with changing
from position mode to position servo mode on the robot. Might want to change
line 738 in Bolide_Y-01_Default_v1.3.0.ino to use `SetPositionS_JOG`. If that 
doesn't work, will want to try modifying what happens in the `A1_16_SetPosition` 
function in `A1_16.cpp` file, specifically changing to servo mode.

It would also be possible to try out speed control, but that sounds extra dangerous
