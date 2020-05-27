# pylint: skip-file
# This is an old example in a library, no need to worry about it

from __future__ import print_function
from serial_coms import SerialCom
from functools import partial

message_received = [False]  # needs to be a list so we can pass by reference


def data_handler(received, *data):
    print("received as ints:")
    print(data)
    print("received as string (may be nonsense):")
    print("".join(map(chr, data)))
    received[0] = True


# bind the first argument to the local var
dh = partial(data_handler, message_received)

bob = SerialCom('COM3', dh)


bob.sendData('how about a string')
while not message_received[0]:  # run until we get a message back
    bob.receiveData()

bob.sendData([1, 2, 9, 40, 255, 235, 0, 244, 255])
message_received[0] = False
while not message_received[0]:  # run until we get a message back
    bob.receiveData()
