# pylint: skip-file
# This is a library file, no need to worry about it
"""Module to communicate over serial connections"""

import serial


class SerialCom:
    """Class for managing serial connections with variable
    length messages and error checking"""

    def __init__(self, port, data_handler, baud_rate=9600, write_timeout=1):
        """Constructs the Serial Communicator

        Args:
            port (string): The port on which to connect
            data_handler (func): A function to handle receiving a complete data
                                 message. Must take a single byte array as an
                                 argument.
            baud_rate (int): The baud rate at which to run
        """
        self.ser = serial.Serial(port, baud_rate, write_timeout=write_timeout)
        self.start_byte = 254
        self.end_byte = 255
        self.reserved_byte = 253
        self.reserved_dict = bytearray([253, 254, 255])
        self.in_message = False
        self.next_reserved = False
        self.write_pos = 0
        self.data = None
        self.first_length_byte = 0
        self.second_length_byte = 0
        self.data_handler = data_handler

    def __del__(self):
        """Destructor fo the object. Closes the serial port"""
        self.ser.close()

    def sendData(self, data):
        """Sends data

        Args:
            data: The data which you wish to send. Can be a bytearray, string,
                  or list of integers 0-255
        """
        if isinstance(data, bytearray):
            byte_data = data
        elif isinstance(data, str):
            byte_data = bytearray(data, 'ascii')
        else:
            byte_data = bytearray(data)
        clean_byte_data = bytearray()
        for val in byte_data:
            if val in self.reserved_dict:
                clean_byte_data.extend(
                    [self.reserved_byte, self.reserved_dict.index(bytearray([val]))])
            else:
                clean_byte_data.extend([val])
            # todo: make this more efficient
        first_length_byte = (len(byte_data) >> 7) & 0x7f
        second_length_byte = len(byte_data) & 0x7f
        header = bytearray(
            [self.start_byte, first_length_byte, second_length_byte])
        footer = bytearray([self.end_byte])
        message = header+clean_byte_data+footer
        self.ser.write(message)

    def receiveData(self, iterations=999999):
        """Receives data from the other side. Will receive as much data as is
           waiting or however many iterations, whichever is smaller. This should
           be called in a loop.

            Args:
                iterations (int): The maximumm number of times to loop through,
                                  receiving data
        """
        while self.ser.in_waiting and iterations:
            just_read = self.ser.read()
            just_read_int = int.from_bytes(just_read, byteorder='little')
            if just_read_int == self.start_byte:
                if self.in_message:
                    raise Exception(
                        'Received a start command when already in a message')
                self.in_message = True
                self.next_reserved = False
                self.write_pos = -2
            elif just_read_int == self.end_byte:
                if not self.in_message:
                    raise Exception(
                        'Received an end command when not in a message')
                elif self.write_pos == self.expected_data_length:
                    self.data_handler(*self.data)
                else:
                    raise Exception(
                        'Received end command prior to expected end of message')
                self.in_message = False
            elif self.in_message:
                if just_read_int == self.reserved_byte:
                    if self.next_reserved:
                        self.in_message = False
                        raise Exception(
                            'Received back to back reserved byte flags')
                    self.next_reserved = True
                elif self.write_pos == -2:
                    self.first_length_byte = just_read
                    self.write_pos += 1
                elif self.write_pos == -1:
                    self.second_length_byte = just_read
                    self.expected_data_length = (
                        (int.from_bytes(self.first_length_byte,
                                        byteorder='little') & 0x7F) << 7) | (int.from_bytes(
                                            self.second_length_byte, byteorder='little') & 0x7F)

                    self.data = bytearray(self.expected_data_length)
                    self.write_pos += 1
                else:
                    if self.next_reserved:
                        self.next_reserved = False
                        if just_read_int >= len(self.reserved_dict):
                            raise IndexError(
                                'Received a reserved index out of range')
                            self.in_message = False
                        else:
                            just_read_int = self.reserved_dict[just_read_int]

                    if self.write_pos >= len(self.data):
                        raise IndexError(
                            'Tried to write outside of data array')
                    else:
                        self.data[self.write_pos] = just_read_int
                        self.write_pos += 1
            iterations -= 1
