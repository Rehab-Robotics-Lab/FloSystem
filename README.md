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
