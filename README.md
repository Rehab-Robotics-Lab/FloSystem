# flo_head

This is a library for controlling Lil'Flo's face. There
are components for both the onboard arduino and the oncomputer
ROS nodes.

## Expected System:
- A computer with ROS Kinetic installed
- A teensy 3.2 Microcontroller
- An adafruit 2039 LED matrix or similar (mouth)
- Two adafruit 959 LED matricies or similar (eyes)
- Arduino installed
- [Teensyduino installed](https://www.pjrc.com/teensy/td_download.html#linux_issues)

## Setting Up:
1. Set address for eyes and mouth: The left eye should be
   soldered on pad A0. The right eye should be unsoldered. The
   mouth should be soldered on pad A1
2. Mount the face. The text should face upright when viewed from
   behind.
3. Plugin: All elements should have their SCL port attached to
   SLC0 (pin 19) on the Teensy Board (refer to Teensy board
   pin layout diagram). All elements should have their SDA port
   plugged into SDA0 on the teensy (port 18). All boards should
   have power connted to the Vin port on the Teensy and ground
   connected to the ground port on the teensy.
4. Plug the Teensy into the computer, open
   `<this repo>/teensy.ino`, if necessary change the
   board in the arduino IDE to be a teensy 3.2, and upload the
   code.
4. Clone or link this repo into a catkin ws. Instructions here: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
4. Build the catkin workspace using `catkin_make` from the catkin_ws directory
5. Navigate to `<this repo>/teensy/src/serial_coms/computer/python/serial-coms/`
   and run `python setup.py install --user`
6. Setup [UDEV Rules to make the face always have the same name](https://unix.stackexchange.com/a/183492):
    1. Run `python -m serial.tools.list_ports` to see which ports
       are connected. Then plug in the device and run it again,
       the new port is the one which you are connected to
    2. Run `udevadm info --name=/dev/ttyACM0 --attribute-walk`
       with the name matching the port which you found in the
       previous step. Find the device with manufacturer
       "Teensyduino". Note the idVendor, id Product, and serial
       number.
    3. Edit file `/etc/udev/rules.d/99-usb-serial.rules` (create if
       necessary) to have:
       `SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", ATTRS{serial}=="XXXXXXX", SYMLINK+="flo_face"`
    4. Load the rules: `sudo udevadm trigger`
    5. You can check that it worked by running: `ls -l /dev/flo_face`
    6. You may need to `sudo chmod 666 /dev/flo_face`


## Running:
1. Run a roscore
2. Open a new terminal window
3. Run `roslaunch flo_face base.launch`
4. Open a third terminal window
### Keyboard
5. You can now run `rosrun flo_face keyboard_teleop.py` to operate the face with
   your keyboard.
### Design study
6. You can run `rosrun flo_face experiment_manager.py`


## Dependencies:
- https://github.com/adafruit/Adafruit_LED_Backpack
- https://github.com/adafruit/Adafruit-GFX-Library
- https://github.com/Rehab-Robotics-Lab/serial_coms

## Communications to Microcontroller
Messages will have the following format:

| item to be updated | binary array of the update |
|:------------------:|:--------------------------:|
| 0 - Mouth          | 16 bytes                   |
| 1 - Left Eye       | 8 bytes                    |
| 2 - Right Eye      | 8 bytes                    |

Note that the array going to the face
