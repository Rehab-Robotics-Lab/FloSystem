# LilFloSystem
This is going to be the master repository for all of the Lil'Flo remote control, local control, messaging, etc.

## Dependencies
- ROS
- pyserial 

## Setup

### NUC
1. Setup Linux, Either Ubuntu or Lubuntu
    1. Make sure to connect to a network and update everything
    2. Make sure to set the system to login automatically
1. Clone this repo somewhere, like `~/Documents/git/LilFloSystem`
2. [Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation)
    1. Be sure to setup ROS to load in your bashrc by adding: `source ~/catkin_ws/devel/setup.bash`
3.  Setup the network in the bashrc. It is likely that you will be on a network that changes your IP address frequently, so that needs to be handled. 
    1. Type `ifconfig` and get the name of the wifi adapter
    2. Add this line to `~/.bashrc`: `function ifip { /sbin/ifconfig $1 | grep "inet addr" | awk -F: '{print $2}' |awk '{print $1}'; }`
    3. Setup the IP address in the bashrc: ``export ROS_IP=`ifip <name of wifi adapter, source /opt/ros/kinetic/setup.bashex: wlp3s0>` ``
    4. Set the ROS Master to be the local machine in the bashrc: `export ROS_MASTER_URI=http://localhost:11311`
3. Create a catkin workspace, ex: `mkdir ~/catkin_ws/src -r` #TODO: actually try that command
    1. Load the catkin ws in your bashrc by adding: `~/catkin_ws/devel/setup.bash`
4. Link the repo into the catkin ws, ex: `cd ~/catkin_ws/src && ln -s ~/Documents/git/LilFloSystem`
    - Why do we use links? It allows us to easily remove the code from the catkin workspace
       without removing it from our computer. We simply delete the link. 
5. Make the code: `cd ~/catkin_ws && catkin_make`

#### Install Various Dependencies:
1. pip install pyserial --user

#### Optional Things to Make Life Easier
1. Enable SSH, so you can actually connect to the robot
2. Enable VNC, so that you can see what is on the robot
3. Install RMATE to allow you to edit code over SSH from a remote text editor:
    ```bash
    sudo wget -O /usr/local/bin/rcode \
    https://raw.github.com/aurora/rmate/master/rmate
    chmod a+x /usr/local/bin/rcode
    ```

### Lighting Control Teensy

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
