# LilFloSystem
This is going to be the master repository for all of the Lil'Flo remote control, local control, messaging, etc.

## Developing
Take a look at `templateSyncConfig.lua` to learn how to sync your work over to the robot.
.#TODO: Need to figure out how to automatically open tabs and windows, etc.


## Dependencies
- ROS
- pyserial

## Setup

### Network
You will need a router to get it all working. There is a Cisco Linksys E1200 that we are
using. Plug in one of the switching Ethernet ports to the port on your development computer.
Setup with:
- SSID: flo-net
- Broadcasting: off
- PWD: xxxxx
- type: WPA2
- router name: flo-net-host

You then need to use the router to share the internet through your development machine:
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
2. ssh into the robot and run the install script
3. Add the includes to the bashrc: `echo 'source ~/catkin_ws/src/LilFloSystem/bash_includes' >> ~/.bashrc`

### Dev Computer
If you would like to have a separate development computer, which would make a
lot of sense, you can mostly follow the instructions for the NUC from above.
However, you will need to change the ROS Master URI to point to the robot which
you want to work with. Also, note that when loading ROS, you change things like
your Python path, which means you probably only want to load the ROS search tree
when actually working in ROS. One way to do that is with something like this in
your bashrc file:

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
a robot. If you want to work in ROS treating your local machine as the robot,
simply type `connect_to_robot 0` in a shell and you are set, ROS will be loaded
and the ROS Master URI will be set to the local host. You can edit and add to the
list of robots to connect to, changing the IP address to match the robot. If
your robot doesn't have a static IP address, this isn't going to work too well.
The way this is setup, calling `connect_to_robot #` will automatically ssh into
the robot with rmate set to route back to allow remote text editing. You may
find that this needs to be altered for your use case.

If you would like to take advantage of the rmate connection, you can use textmate
or VS Code ([instructions](http://michaelsobrepera.com/guides/vscode.html))
