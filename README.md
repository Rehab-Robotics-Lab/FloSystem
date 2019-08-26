# LilFloSystem
This is going to be the master repository for all of the Lil'Flo remote control, local control, messaging, etc.

## Developing
Take a look at `templateSyncConfig.lua` to learn how to sync your work over to the robot.
.#TODO: Need to figure out how to automatically open tabs and windows, etc.

## Dependencies
- ROS
- pyserial

## Setup

### Dev Computer
You will need to have a development and control computer

1. Setup Ubuntu
    1. Make sure to connect to a network and update everything
    2. Install lsyncd: `sudo apt install lsyncd`
2. Clone this repo into `~/Documents/git`
3. Run `./dev_install.sh`
4. Setup your bashrc or zshrc to have ros stuff. This is somewhat dependent on
   what you want, but here are some ideas:

```bash
function ifip { /sbin/ifconfig $1 | grep "inet addr" | awk -F: '{print $2}' |awk '{print $1}'; }

export ROS_IP=`ifip wlp3s0`

function connect_to_robot {
    source /opt/ros/kinetic/setup.bash
    source ~/catkin_ws/devel/setup.bash

    # takes a single number arg representing the robot
    if (( $1 == 3 ))
    then
        export ROS_MASTER_URI=http://158.130.191.68:11311 # Robot 3
        ssh -R 52698:localhost:52698 sr3@158.130.191.68
    elif (( $1 == 4 ))
    then
        export ROS_MASTER_URI=http://158.130.191.69:11311 # Robot 4
        ssh -R 52698:localhost:52698 servicerobot4@158.130.191.69
    elif (( $1 == 0 ))
    then
        export ROS_MASTER_URI=http://localhost:11311
    fi
}
```

This first sets up the local IP address, then gives you a function to connect to
a robot. If you want to work in ROS treating your local machine as the robot
(which will be the use case for this system)
simply type `connect_to_robot 0` in a shell and you are set, ROS will be loaded
and the ROS Master URI will be set to the local host. You can edit and add to the
list of robots to connect to, changing the IP address to match the robot. If
your robot doesn't have a static IP address, this isn't going to work too well.
The way this is setup, calling `connect_to_robot #` will automatically ssh into
the robot with rmate set to route back to allow remote text editing. You may
find that this needs to be altered for your use case.

It is probably useful to edit your hosts file: `sudo nvim /etc/hosts` to have the nuc listed. Add this: `10.42.0.189 flo-nuc`

You can also create a function in your bashrc that allows easier sshing in:

```bash
function ssh-flo {
    ssh nuc-admin@flo-nuc
}
```

### Network
You will need a router to get it all working. There is a Cisco Linksys E1200 that we are
using. Plug in one of the switching Ethernet ports to the port on your development computer.
Setup with:
- SSID: flo-net
- Broadcasting: off
- PWD: xxxxx
- type: WPA2
- router name: flo-net-host

You then need to use the router to share the Internet through your development machine:
- Set router into AP Mode (Access Point Mode)
- Open network settings on the development machine and click edit connections
- Click Add
- Select Ethernet
- name the connection `shared-net`
- Under general, select Automatically connect when available and all users may use network
- Under IPv4 settings select Method: Shared to other computers

You should then be able to connect to the router using the settings setup on the router
at the beginning from the nuc on the robot

### NUC
1. Setup Ubuntu
    1. Make sure to connect to a network and update everything
    2. Make sure to set the system to login automatically
    3. enable ssh: `sudo apt install openssh-server`
1. Use lsyncd with the configuration file (See section on Developing) to copy files over
2. ssh into the robot and run the install script (`robot_install.sh`)
4. You need to setup read/write privileges for all of the USB devices:

#### Assigning the serial devices to have a fixed addresses
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

The naming scheme ends up like this:

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="1582410", SYMLINK+="flo_face", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A104D4GV", SYMLINK+="bolide", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="kobuki_A907BUO2", SYMLINK+="kobuki", MODE="0666"
```

If this is working correctly then the mode should be 666 to allow read/write privileges.
If it isn't working and getting a no access error, then you will need to run
`chmod 666 /dev/XXXX`. Then figure out what is wrong and fix it...


#### Amazon Polly
Polly will need to have access to AWS to work. To set that up, go to the
[IAM Console](https://console.aws.amazon.com/iam)
and click on users. Then select the user you want to give access to.
Click "Create access key" under Security Credentials.
Then in the console on the computer,
[install the AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-install.html)
(This should actually be done by the install script)
Then in the console, run `aws configure --profile flo` and fill in the info from the
IAM console with region set to us-east-1 and output set to json.

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
  and then play the audio by installing: `sudo apt install mpg123` and typing `mpg123 test.mp3`
- you might find that you are getting some sort of server connection errors.
  you can resolve that by running `pip3 install -U boto3`
  (this should now be a part of the install script, but it is commented out)


## Some useful tools:
For monitoring the kobuki, you can use the [kobuki dashboard](http://wiki.ros.org/turtlebot_bringup/Tutorials/indigo/PC%20Bringup).
Run: `rqt -s kobuki_dashboard`.

## Things that may break:

### The web interface just doesn't do anything
check to see if the socket is just closing imediately. If it is, then it may
be that the version of tornado on the server is wrong. Uninstall the version
from pip, then uninstall the version from apt, then reinstall the rosbridge
suite.

### Packages are missing
It probably means you added dependencies since installing. Most dependencies are installed
via rosdep by running:
```bash
rosdep install --from-paths ~/catkin_ws/src/LilFloSystem/ --ignore-src --rosdistro=kinetic -y
```
Some dependencies can't be installed by rosdep and should be installed by the install scripts.
In that case, you should look at the most recent commits into git for the install scripts
and try to figure out which commands need to be run.

### Weird stuff says services don't exist
Try running catkin_make on the catkin ws

### REALSENSE has version mismatches and stuff
apt update and upgrade
Go into catkin_ws/src/realsense-ros
run:
```bash
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
```
cd up to catkin_ws and run catkin_make floowed by catkin_make install
