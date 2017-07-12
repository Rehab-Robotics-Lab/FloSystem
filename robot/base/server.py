import asyncio
import websockets

import serial
import time

import json

# Script to move around the iRobot Create from websocket interfce
# Author: Michael Sobrepera
# Date: 2-Sept-2016

# To find the port run:
# python -m serial.tools.list_ports

def num2hex_hl(num):
    """Converts int to 16bit 2's complement and converts into its high and low bytes in hex.

    Args:
        num (integer): the number to be converted

    Returns: the high byte, the low byte
    """
    high, low = divmod(num if num >= 0 else (1 << 16) + num, 0x100)
    return high, low

class Create1(object):
    """A class to communicate with a create1 robot.
    
    Attributes:
        _serial: The serial connection used to talk to the robot
        current: The current command set which was sent to the robot
        simulate: Whether the system is in simulation mode
        port: The port which the robot is connected to
        """

    def __init__(self, port, max_speed=400,simulate=False):
        """Constructor for the object.

        Opens the connection to the robot, unless in simulation mode.

        Args:
            port (string): The serial port to connect to on the local machine.
            simulate (boolean): Whether to simulate the connection and just
                                print to the console.
            max_speed (int): The maximum speed of the wheels in mm/s
        """
        self.current = 'stop' # the current command state
        self.simulate = simulate
        self.port = port
        self.max_speed = max_speed
        self._serial = None
        print('begining init')
        if self.simulate:
            print('[simulation]\tconnected')
        else:
            print('connecting to robot')
            self._serial = serial.Serial(port,57600)
            self.send([128,131]) # start interface and put in safe mode
            print('connected to robot')
        print('init complete')
        self.move(0,0)
        print('set zero speed')

    def __del__(self):
        """Deconstructs the object by stopping the robot and closing the serial
        port."""
        print('begining delete')
        self.move(0, 0)
        if self._serial:
            self._serial.close()
        print('done deleting')

    def move(self, left, right):
        """Convert wheel speeds into a message for the create 1 platform.

        Args:
            left (float): The left wheel speed, as a 0-1 value
            right (float): The right wheel speed, as a 0-1 value
        """
        left = int(left*self.max_speed)
        right = int(right*self.max_speed)
        left_high, left_low = num2hex_hl(left)
        right_high, right_low = num2hex_hl(right)
        msg = [145,right_high, right_low, left_high, left_low]
        self.send(msg)

    def send(self, input):
        """Send the input command over serial, or print if in simulation mode.

        Args:
            input (vector of numbers): Command sequence to send to robot
        """
        if input!=self.current:
            current = input
            if self.simulate:
                print('[simulation]\tsending: {}'.format(input))
            else:
                for val in input:
                    try:
                        print('\t\tsending:\tval: {}\t{}'.format(val, 
                            serial.to_bytes([val])))
                        self._serial.write(serial.to_bytes([val]))
                    except ValueError:
                        print("invalid value passed: {}".format(val))

    def receive(self, message):
        """Take in a json message indicating the joystick values, parse
        that to movement values for the differential drive robot and move the
        robot.

        The output
        will be such that for abs(y)<.1, all motion will be pure rotation, with
        the magnitude of rotation dicatated by the x value. Positive x will
        yield clockwise rotation. Negative x will yield counter clockwise
        rotation. Values which have abs(y)>.4 will be in a standard
        forward/backwards motion space. In this space, the y will determine the
        maximum available speed. The x value will determine the percent being
        delivered to each wheel. Greater than zero, the right wheel will get
        100%, lesss than zero, the wheel will linearly scale to 0% at x=-1. The
        left wheel will be converse. Between the two limits, the rotational and
        linear components will blend by a linear factor. In order to make the
        blending continuous, in the negative y space, the results will be
        negative and flipped across the y axis.

        Args:
            message: A JSON string representing the status of a joystick with structure:
                root - clicked - [t/f]
                     - pos_normal - x - [num [0-1]]
                                  - y - [num [0-1]]
        """
        msg_dict = json.loads(message)
        if not msg_dict['clicked']:
            self.move(0,0)
        else:
            x = msg_dict['pos_normal']['x']
            y = msg_dict['pos_normal']['y']
            mag_x = min(abs(x), 1)
            mag_y = min(abs(y), 1)

            # calculate rotational component
            if x<0:
                rotation = (-mag_x, mag_x)
            else:
                rotation = (mag_x, -mag_x)

            # calculate linear component
            if y>0:
                if x<0:
                    linear = (mag_y*(1-mag_x), mag_y)
                else:
                    linear = (mag_y, (1-mag_x)*mag_y)
            else:
                if x<0:
                    linear = (-mag_y, -mag_y*(1-mag_x))
                else:
                    linear = (-(1-mag_x)*mag_y, -mag_y)

            # calculate blending
            blend = min(max((-10/3)*mag_y + 1+1/3,0),1)

            result_left = linear[0]*(1-blend) + rotation[0]*blend
            result_right = linear[1]*(1-blend) + rotation[1]*blend

            self.move(result_left, result_right)
            if y<0.1:
                blend = 1
            elif y>0.4:
                blend = 0

    async def handler(self, websocket, path):
        """Handle inputs from the websocket. 
        
        Acts a handler for the python websockets library.
        
        Args:
            websocket (websoket): The websocket from which to read from 
            path: The URI to whomever is talking to us
        """
        while True:
            message = await websocket.recv()
            print("< {}".format(message))
            self.receive(message)


# start usb server
port = 'COM3'
robot_base = Create1(port)



start_server = websockets.serve(robot_base.handler, 'localhost', 5678)

# start websocket server server
asyncio.get_event_loop().run_until_complete(start_server)

# run event loop
asyncio.get_event_loop().run_forever()