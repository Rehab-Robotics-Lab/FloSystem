# Flo_ Humanoid
This has the ros packages necessary to talk to the bolide robot

## Dependencies:
- pyqtgraph: `pip install pyqtgraph`
- ros kinetic

## Assigning the bolide to have a fixed address
Setup [UDEV Rules to make the face always have the same name](https://unix.stackexchange.com/a/183492):

    1. Run `python -m serial.tools.list_ports` to see which ports
       are connected. Then plug in the device and run it again,
       the new port is the one which you are connected to
    2. Run `udevadm info --name=/dev/ttyACM0 --attribute-walk`
       with the name matching the port which you found in the
       previous step. Find the device with manufacturer
       . Note the idVendor, id Product, and serial
       number.
    3. Edit file `/etc/udev/rules.d/99-usb-serial.rules` (create if
       necessary) to have:
       `SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", ATTRS{serial}=="XXXXXXX", SYMLINK+="flo_face"`
        `SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="bolide"`

    4. Load the rules: `sudo udevadm trigger`
    5. You can check that it worked by running: `ls -l /dev/flo_face`
    6. Chmod 666 the port so you can actually write to it...
