# Flo_ Humanoid
This has the ros packages necessary to talk to the bolide robot

## Dependencies:
- pyqtgraph: `pip install pyqtgraph`
    - This is only needed if you want to run the read from bolide code and get a nice graph. Not generally needed
- ros kinetic
    - Just do the full desktop install and you are good
- numpy
    - Installed via rosdep
- matplotlib
    - installed via rosdep

### Installing dependencies
All of the critical dependencies can be installed using rosdep. You will need to:
`sudo apt install python-rosdep`

Then go into your catkin workspace and run:
`rosdep install --from-paths src --ignore-src -r -y`

audio: `sudo apt install ros-kinetic-audio-common`

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
       `SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", ATTRS{serial}=="XXXXXXX", SYMLINK+="flo_face", MODE="0666"
        `SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="bolide"`

    4. Load the rules: `sudo udevadm trigger`
    5. You can check that it worked by running: `ls -l /dev/flo_face`
    6. Chmod 666 the port so you can actually write to it...

### Amazon Polly
Polly will need to have access to AWS to work. To set that up, go to the [IAM Console](https://console.aws.amazon.com/iam)
and click on users. Then select the user you want to give access to. Click "Create access key" under
Security Credentials. Then in the console on the computer, [install the AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-install.html).
Then in the console, run `aws configure --profile flo` and fill in the info from the IAM console with region set
to us-east-1 and output set to json.

A few notes:
- you should probably restrict what the user that is getting access can do. The best option is
  to set the permissions to AmazonPollyReadOnlyAccess
- You can have other users configured. By changing the --profile, you control the name of the user
  if no profile is specified, then the information is saved to a default
- If you ever need to revoke permissions for a user, that can be done from the IAM Console by
  deactivating the user's Access Keys.
- TODO: I would like to find a way to limit how many calls can be made by a user.
- To check whether polly is available given the profile, run `aws polly help --profile flo`.
  If you get back a list of commands to run on polly, you should be good.
- To fully test, can run: `aws polly synthesize-speech --output-format mp3 --voice-id Ivy --text 'hello, this is a test' --profile flo test.mp3`
- you might find that you are getting some sort of server connection errors.
  you can resolve that by running `pip3 install -U boto3`

## Getting video to work
`sudo apt install ros-kinetic-usb-cam`
Then run node with rosrun usb-cam usb_cam_node

`sudo apt install ros-kinetic-webrtc-ros`
`rosrun webrtc_ros webrtc_ros_server__node`
