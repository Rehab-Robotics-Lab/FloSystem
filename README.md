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

#### Additional install targets

There are a few things that aren't installed by this script that you may want:

1. AWS CLI, you can install this with `aws_install.sh`.
   You will need the AWS CLI to use the TTS engine. The install script will ask you
   for your AWS login info, follow the instructions in [Amazon Polly](#amazon-polly)
2. Node, which is needed to develop and run the websever. You can install directly
   (better for the robot) using `node_install.sh` or with nvm (better for development)
   using `nvm_install.sh`.

### Network

Some helpful commands (you will need to install `net-tools` for some):

- `nmcli device status` tells you what your network devices are doing. `ip link show` does something similar
- `nmcli connection show` tells you waht networks are on your list (not the same as what is currently available, this is what you have connected to and saved)
- `nmcli` gives a general overview of net status
- `ip a` prints out a ton of info with ip and mac addresses
- `netstat -i` gives a clean print out of how much traffice each adapter is seeing
- `lspci | egrep -i --color 'network|ethernet'` to get which physical network adapters are available
- `cat /proc/net/dev` will show the physical cards even those that aren't installed
- `sudo lshw -class network` gives hardware details
- `lsusb` will show the installed usb devices

#### New Way (tp-link AC750)

1. Buy a tp-link ac750 travel router (TL-WR902AC)
2. Plug it into power and se tthe mode switch to share hotspot
3. Go through the quick setup to connect to something
4. Set the broadcast network name as flo-net for both 2.4 and 5 Ghz
5. Set security to WPA2-PSK/AES with password floiscool#01
6. Under the network tab, click LAN and set the IP Address to 10.42.0.1
7. Under DHCP set the range to 10.42.0.100 - 10.42.0.199
8. Under IP & MAC binding, set ARP Binding to be enabled
9. Click into the ARP List and select the nuc, set it to be bound on 10.42.0.189
10. Under DHCP/Address Reservation, add the MAC and IP address that you set to bind
11. click under system tools and change the password
12. connect everything up.

When you enter somewhere new, you will need to connect to the local network. TO do this:

1. Turn on the router
2. navigate to 10.42.0.1
3. select the quick setup and click through to setup

Some thoughts on this:

- Using 5GHz gives better bandwidth, but that isn't always what you want.
  2.4GHz gives better range. So you might want to use 2.4 GHz instead.
  To do that, just name the 5ghz connection something else.
- Using the network share means the chip is doing extra. It might be faster/stronger
  if it came off of the ethernet.
- The router can be placed between the robot and laptop, either using wifi or a cable.

#### Old Way (Router hooked up through ethernet)

You will need a router to get it all working. There is a Cisco Linksys E1200 that we are
using. Plug in one of the switching Ethernet ports to the port on your development computer.
Note for Ubuntu 18 you may need to run `nm-connnection-editor`.
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

#### SSH-Keys

You will now need to setup SSH Keys. Just follow [this guide](https://www.digitalocean.com/community/tutorials/how-to-set-up-ssh-keys-on-ubuntu-1804)

### NUC

1. Setup Ubuntu
   1. Make sure to connect to a network and update everything
   2. Make sure to set the system to login automatically
   3. enable ssh: `sudo apt install openssh-server`
1. Use lsyncd with the configuration file (See section on Developing) to copy files over
1. ssh into the robot and run the install script (`robot_install.sh`)
1. add a symlink to make running easier: ssh in and type `ln -s ~/catkin_ws/src/LilFloSystem/robot_tmux_launcher.sh`.
1. You need to setup read/write privileges for all of the USB devices:

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
       `SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", ATTRS{serial}=="XXXXXXX", SYMLINK+="flo_face", MODE="0666"`

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

#### USB Speaker

##### Not sure if this is needed:

You need to make the usb speaker the default device:
sudo vim /usr/share/alsa/alsa.conf

change:

```bash
defaults.ctl.card 0
defaults.pcm.card 0
```

to:

```bash
defaults.ctl.card 1
defaults.pcm.card 1
```

then restart

#### Realsense Cameras

The realsense cameras seem to hold a lot of settings on board.
For now, you need to plugin the camera and run `realsense-viewer`.
For now, set the system to high accuracy with the laser projector set to laser.

### Getting WebRTC ROS installed

This will need to happen wherever you are running the webserver:

1. Clone the dev branch either into the catkin_ws/src folder or somewhere else and link it in for the repo: https://github.com/RobotWebTools/webrtc_ros
2. Add a file at `webrtc_ros/webrtc` named `CATKIN_IGNORE`, leave it empty
3. Build the catkin workspace as normal
4. make sure that at `LilFloSystem/flo_web/web_app/public` you link (`ln -s`) to `~/catkin_ws/src/webrtc_ros/webrtc_ros/web`. This should already be present.

The prior instructions give you the latest version of the package. This is needed
as the build farm version is not being updated quickly. However if eventually
the build farm version is used (likely via a rosdep dependency), then the link
will need to be changed to point to `/opt/ros/<distro name>/share/webrtc_ros/web`

## Running

1. ssh into the robot: `ssh nuc-admin@<ip addr>`
2. Run the tmux launch script: `./robot_tmux_launcher.sh`
3. Attach to tmux: `tmux a`

### for development

There is a simulator you can use under the `flo_core` package: `roslaunch flo_core flo_sim.launch`

You will also need to launch the webserver by going to `flo_web/web_app` and running `npm start`.
For development, you may want to run the browser on your local machine with `npm chrome`

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
sudo apt update -y && sudo apt upgrade -y
rosdep update
rosdep install --from-paths ~/catkin_ws/src/LilFloSystem/ --ignore-src -r -y
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

It is also possible that you need to update the camera's firmware.
To do this, open realsense-viewer with the camera connected to a computer with an available
GUI.
You will be prompted to update the firmware.

### Stuff just won't build

All kinds of things could cause this. The first thing to try it to clean
the build and reinstall it:

```bash
cd ~/catkin_ws
catkin_make clean
catkin_make
catkin_make install
```

### There is no audio playing

The first thing to do is test the speaker by running `speaker-test`
Then hop into Alsa Mixer `alsamixer`, press F6 and select the USB Device, but this can only change the volume.
It is also possible that the wrong audio device is selected. To fix that, you will need to select the audio device.
List the available devics by: `pacmd list-sinks|grep index` and then set the one you want with `pacmd set-default-sink <sink_name|index>`
You can also set the volume here.

### Videos don't show up when running

There are a few possible problems:

1. things only seem to work in chrome for now, so use that.
2. You may not have your devices enabled because the source is non-secure. Here is what to do:
   a. goto: chrome://flags/#unsafely-treat-insecure-origin-as-secure
   b. fill in with: `http://10.42.0.189:5000,http://10.42.0.189:3000,http://10.42.0.189:9090,http://10.42.0.189:9091`
   c. change to enabled

### Realsense cameras are capturing at a low frame rate

This can happen if the room you are working in is poorly lit.
The cameras have an autoexposre setting and if the exposures need to be
really long, then the camera will slow down the framerate to tolerate that.
You can either turn off the autoexposure and set it to under expose or brighten the room.
