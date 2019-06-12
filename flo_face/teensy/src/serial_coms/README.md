# serial_coms
A set of libraries to facilitate data transfer over serial communication lines. It will deliver the contents of an arbitrary length bit array from one side to the next. Initially the library will only run in unverified mode (think UDP) where data is sent without regard for the receiving end. Eventually a verified mode (think TCP) will be made, where data is verified to have been accurately received via checksum returns and where data can only be sent once the prior message is received. 

Our lab consistently needs to communicate between microcontrollers and computers. The simple method of sending one way bit streams isn't sufficient for most of what we do. So this library will be implemented to send richer data in both directions with optional error checking on both ends. In theory it should also work for microcontroller to microcontroller and computer to computer communications. Initially it will be developed for arduino in c++ (in a way that is easily portable to other microcontroller architectures) and in python for pc. It would make sense to also implement it in c++ or go on PC at some point.

## Level 1 Protocol
The first level of the protocol simply transmits data back and forth in a defined message format

| Start Marker | Total Data Length | Data        | End Marker |
|:------------:|:-----------------:|:-----------:|:----------:|
| 1 byte       | 2 byte            | 1-16k bytes | 1 byte     |
| 254          |                   |             | 255        |

### Message Length
The 2 bytes in the message length field will each be sent with a leading zero (0xxxxxxx) (this prevents collisions with the start and end markers). To obtain the actual message length, the zero will be stripped and the remained will be concatenated into a 14 bit unsigned int. This yields a maximum data size of 16383 bytes. 

### Reserved Bytes
Reserved value access: 253
253 000 => 253
253 001 => 254
253 002 => 255

Any time a reserved byte (253-255) is present in the data, it will be replaced by the appropriate two bytes which represent it. This prevents collisions between data values and the end marker

### Message Type, Handshakes, Message ID, etc...
Any additional infrastructure such as message IDs, handshakes, delivery receipts, etc.. will be implemented at a higher level as part of the data section of the message. 

## References / Acknowledgements
Based heavily on the work by Robin2: 
[Demo of PC-Arduino comms using Python](http://forum.arduino.cc/index.php?topic=225329)

## What port am I operating on?
python -m serial.tools.list_ports