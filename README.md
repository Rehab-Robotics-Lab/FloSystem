# LilFloSystem

![Check Syntax](https://github.com/Rehab-Robotics-Lab/LilFloSystem/workflows/Check%20Syntax/badge.svg)
![Install & Build](https://github.com/Rehab-Robotics-Lab/LilFloSystem/workflows/Install%20&%20Build/badge.svg)

This is the the master repository for all of the Lil'Flo remote control, local
control, messaging, etc.

In this README you will find everything you need to work with and develop on the
Lil'Flo platform.

You may be interested in [papers and posters on the project](https://michaelsobrepera.com/tags/flo)

## License:

Currently all code, documentation, other content, and ideas in this repository is available for you to view.
If you would like to use the code, documentation, other content, or ideas in this repository, please reach out to mjsobrep@seas.upenn.edu.

### Why so restrictive?

We are waiting on approvals from the owners of the repository (The University of Pennsylvania) to approve a less restrictive license.

## Contents

- [WebServer Setup](#webserver-setup)
- [Development Computer Setup](#dev-computer)
- [Network Setup](#network)
- [NUC Setup](#nuc)
  - [UDEV for USB fixed Addresses](#udev)
  - [USB Speker](#usb-speaker)
  - [Realsense Cameras](#realsense-cameras)
- [Amazon Polly Setup](#amazon-polly)
- [WebRTC ROS Setup](#webrtc-ros)
- [Developing on The System](#developing)
  - [Simulator](#simulator)
- [Running Code](#running)
- [Things That May Break](#break)
  - [The Web Interface Doesn't Respond](#broken-web)
  - [Packages are missing](#missing-packages)
  - [Services Don't Exist](#broken-services)
  - [Realsense has version mismatches](#broken-realsense)
  - [ROS won't build](#broken-build)
  - [No audio plays](#broken-audio)
  - [No videos on web](#broken-web-video)

## WebServer Setup

Setting up the webserver is a totally different issue from setting up the robot.
Everything runs in docker. Let's walk through real quickly what is going on:
When anything outside hits the webserver stack, they are going to route through
Nginx, which is listening on both ports 80 and 443. The definition for Nginx depends on the environment, production or development, and is
defined in `flo_web/nginx-prod.conf` and `flo_web/nginx-dev.conf`
respectively. In production, Nginx will serve the static front end
files itself. In development create-react-app (via nodejs) will serve
them so that the developer has hot reloading. The Nginx docker image
is defined in `flo_wb/Dockerfile` where the front end is first compiled
and then added into Nginx.

When in development, the create-react-app runtime is defined in
`flo_web/web_app/Dockerfile`.

For both production and development, the backend socket server runtime
is defined in `flo_web/web_server/Dockerfile`.

There is a base docker-compose file in the root. This file defines
the operations during production. There is also a docker-compose
override file, which defines operations during development.
This is done by changing the commands which run in the docker images,
changing what volumes are mapped to bring in code and config files,
and adding in the front end server.

You should create a file in `LilFloSystem/certs/session-secret.env`
with contents: `SESSION_SECRET=<cryptographically random value>`

You should create a file in `LilFloSystem/certs/coturn.env`
with contents:
```conf
COTURN_SECRET=<cryptographically random value>
SITE_ADDR=<the site address, ex: lilflo.com>
```

####To run in development:

1. Go to the root of this repo
2. `docker-compose build`
3. `docker-compose up`
   In theory, you shouldn't have to rebuild for any changes.
   If you change the Nginx dev config, you will either need
   to go into the Nginx container and restart it or restart
   the entire docker-compose group (ctrl-c; `docker-compose up`)

#### To run in production:

1. Go to the root of this repo
2. `docker-compose -f docker-compose.yml up -d`
   TODO: [add a production def](https://docs.docker.com/compose/extends/#multiple-compose-files) that sets restart rules and stuff.
3. If you need to take it down, run: `docker-compose down`

### Using docker compose:

Docker compose is nice because it will build, name, and run all of the necessary
images.
This is defined in the `docker-compose.yml` file in the root.

1. Go to the root
2. Run `docker-compose build`
3. Run `docker-compose up`, Ctrl+c to take down
   - To put in background run `docker-compose -D up`
   - To take out of background: `docker-compose down`

### SSL certs

#### Local

We need ssl certs during local development.

1. sudo apt install libnss3-tools
2. Download the latest binary for [mkcert](https://github.com/FiloSottile/mkcert)
3. Change permissions `sudo chmod u+x <name of binary>`
4. Setup certs registry: `<name of binary: mkcert...> -install`
5. Make a certs dir (named `certs`) in the root of this repo and go into it
6. Make new certs: `<name of binary: mkcert...> localhost` you can add other options here if you want to simulate a local domain by putting it in your hosts file, add in the 127.0.0.1 or other localhost aliases, etc. You can even use wildcards. NOTE: be very careful with these, they can really really open you up to security holes in your local computer if shared.
7. rename the certs: `mv localhost-key.pem localhost.key && mv localhost.pem localhost.crt` Of course if your certs are named something else... you get the idea.

#### Server

We use certbot to generate certificates automatically. The certbot certificates
automatically. They expire every so often, so they need to be regularly regenerated.
We have it run once a week on sunday at 2:30AM eastern to regen certs and
3:30AM eastern to restart the server and use the new certs.
The whole system runs based on a [guide](https://medium.com/@pentacent/nginx-and-lets-encrypt-with-docker-in-less-than-5-minutes-b4b8a60d3a71)

### Deploying to Linode

Linode is small, easy to use, and affordable.

1. Setup a node on linode. The small size should be fine. When setting up, setup an ssh-key to make your life easier. Use the latest Ubuntu LTS.
2. Setup the A/AAA record to work with your domain name and have your register use the linode domain servers.
3. Install and setup firewall:
   1. `apt install ufw`
   2. `ufw default allow outgoing`
   3. `ufw default deny incoming`
   4. `ufw allow ssh`
   5. `ufw allow http`
   6. `ufw allow https`
   7. `ufw allow 49152:65535/udp`
   7. `ufw allow 49152:65535/tcp`
   7. `ufw allow 3478/tcp`
   7. `ufw allow 3478/udp`
   7. `ufw allow 5349/tcp`
   7. `ufw allow 5349/udp`
   7. `ufw enable`
   8. Check status with `ufw status`
4. [Setup unattended updates](https://help.ubuntu.com/lts/serverguide/automatic-updates.html): `apt install unattended-upgrades`
5. Clone this repository
6. Install docker-compose: `apt install docker-compose`
7. Go into the repo root and run `docker-compose -f docker-compose.yml -f docker-compose.prod.yml build`
8. Run `docker pull certbot/certbot`
9. Setup certificates by running `./init-letsencrypt.sh`
10. Run `docker-compose -f docker-compose.yml -f docker-compose.prod.yml up -d`
11. Monitor with `docker ps`, `docker stats`, and `docker-compose logs -f`

### Deploying to Heroku

Discovered that on heroku, each individual docker image/service needs its own dyno and the way that they talk between each other doesn't work really well.
So we are not going with heroku.
But I am leavin this here for future ref

Heroku is a nice quick way to deploy. They provide a free hobby dyno to students,
so using that. The Heroku-Redis and Heroku-Postgres serve the need for in system
messaging, in system memory, and database. In theory heroku will handle keeping
those systems safe.

1. [Install the Heroku CLI](https://devcenter.heroku.com/articles/heroku-cli#download-and-install)
2. Login to Heroku CLI: `heroku login`
3. Create an app: `heroku create <app-name>`. Note that the app name is in a
   global namespace for all heroku apps, so for example `flo-control-system`
   is alredy taken
4. If you don't want to be storing plain text passwords (you don't) you need to
   [setup a credential helper](https://docs.docker.com/engine/reference/commandline/login/#credentials-store)
   1. Install the helper, download
      [the latest release](https://github.com/docker/docker-credential-helpers/releases)
      and put it in your path. Easiest to just drop in ~/bin and go change the permissions to
      be runnable (`chmod u+x <filename>`)
   2. To use pass:
      1. Install pass and gnupg: `sudo apt install pass gnupg`
      2. Make a gpg key: gpg --full-gen-key
      3. Initialize pass: pass init <emailaddr>
   3. Setup the docker config by editing ~/.docker/config.json to have:
      `"credsStore": "pass"`
5. login to the container system: `heroku container:login`

## Setup

Throughout this setup, we will assume that you are using a clean install of the
version of linux that is called for. The setup may not tolerate weird tweaks to
the python version, will not work with conda, and will not work with python
virtual environments.

The first think to do is to setup your development computer:

### Dev Computer

You will need to have a development and control computer. At the least you will
need to follow the instructions in the [Network](#network) section. If you want
to actually be writing new code and devloping, you will need all of this:

1. Setup Ubuntu, either 16 or 18
2. Install git: `sudo apt intall git-all`
3. Clone this repo into `~/Documents/git`
4. Run `./dev_install.sh`
5. Setup your bashrc or zshrc to have ros stuff. There an
   [example loader](example-loader) that you can take a look at.
6. If you are going to be testing the text to speech system on your development
   computer, then you need to follow the
   [Amazon Polly Instructions](#amazon-polly)
7. Node, is needed to develop and run the websever. You can install directly
   (better for the robot) using `node_install.sh` or with nvm (better for
   development) using `nvm_install.sh`. If you are working with the web server,
   you will also need to install
8. You will need to [setup the network](#network)

#### Example Loader

Note this was written to run in zsh, there are places where bash may want a
semicolon, but really, why are you using bash?:

This will set the ROS and OS version and also setup a function to get the ip
address of this macine on different NICs. This should run without problems

```bash
if (($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==16));then
    ROS_VERSION="kinetic"
    OS_VERSION="xenial"
    function ifip { /sbin/ifconfig $1 | grep "inet addr" | awk -F: '{print $2}' |awk '{print $1}'; }
    else
    if (($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==18)); then
    ROS_VERSION="melodic"
    OS_VERSION="bionic"
    function ifip { ip -4 addr show $1 | grep -oP '(?<=inet\s)\d+(\.\d+){3}' }
fi
fi
```

You then need the code to actually connect to the robot:

```bash
function connect_to_robot {
    echo "setting up for $1"
    echo "using ros version: $ROS_VERSION"
    echo "on OS: $OS_VERSION"
    source /opt/ros/${ROS_VERSION}/setup.zsh
    source ~/catkin_ws/devel/setup.zsh

    case $(hostname) in
        linPower)
            echo "recognized device Lin Power"
            export ROS_IP=`ifip enp0s31f6`;;
        x220t)
            echo "recognized device x220t"
            export ROS_IP=`ifip wlp3s0`;;
        mjs-mws)
            echo "recognized device MJS-MWS"
            export ROS_IP=`ifip enx98e743e78ee0 || ifip wlp59s0`;;
        *)
            echo "unknown system";
    esac
    # Handle the device not existing and getting something empty:
    [  -z "$ROS_IP" ] && (echo "ROS_IP unset, setting on loopback"; export ROS_IP=`ifip lo`)

    # takes a single number arg representing the robot
    if [ "$1" = "flo" ]
    then
        export ROS_MASTER_URI=http://flo-nuc:11311
        echo "added nuc as master uri"
    elif [ "$1" = "kq" ]
    then
        export ROS_MASTER_URI=http://192.168.1.24:11311
        echo "added kendal's computer as master uri"
    elif (( $1 == 0 ))
    then
        export ROS_MASTER_URI=http://localhost:11311
        echo "connected to local machine"
    fi
    echo "set ROS MASTER URI to $ROS_MASTER_URI"
    echo "set ROS IP to $ROS_IP"
    echo "done setting up ros"
}
```

So now, when you run `connect_to_robot xyz` in the terminal:

1. all of the necessary ROS files will be sourced
2. based on the computer you are on, the correct NIC will be used to get the IP
   address.
   1. Note that for mjs-mws, there are two NICs of interest, that are ordered by
      priority. This allows us to use a wired network if available and if not to
      fall back to the wireless.
   2. If there is no IP address, ie. your not connected to a network, then the
      loopback device will be used.
3. Based on the input given, the ros master will be set to connect to a specific
   robot.
   1. For zero, the local machine will be connected to.
   2. Other options are `flo`, `kq`, `x220t`. You probably don't need all of
      these, but it gives you a nice template of how to work.

Some things to keep in mind:

- You need to adjust the hostnames section. You should use the host name of your
  computer and the network adapter you want. You can find the names of your
  network adapter by typing either `ifconfig` or `ip a` in the terminal
- You will likely need to add a different robot. You can simply use the ones
  that are above as a template. You will need your common language name for the
  robot and its IP address.
- `connect_to_robot 0` will connect to your local robot

### Network

Some helpful commands (you will need to install `net-tools` for some):

- `nmcli device status` tells you what your network devices are doing.
  `ip link show` does something similar
- `nmcli connection show` tells you waht networks are on your list (not the same
  as what is currently available, this is what you have connected to and saved)
- `nmcli` gives a general overview of net status
- `ip a` prints out a ton of info with ip and mac addresses
- `netstat -i` gives a clean print out of how much traffice each adapter is
  seeing
- `lspci | egrep -i --color 'network|ethernet'` to get which physical network
  adapters are available
- `cat /proc/net/dev` will show the physical cards even those that aren't
  installed
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

It also makes sense to setup for tethered sharing, in the [old way](#old-way):

1. Set switch to AP mode
2. connect by wifi and navigate to 192.168.0.1
3. Quick setup set ssid to flo-net and password to floiscool#01
4. Plug in cable
5. Rock and roll

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

You then need to use the router to share the Internet through your development
machine, note that if you are actually in the rehab robotics lab, this is
already done:

- Set router into AP Mode (Access Point Mode)
- Open network settings on the development machine and click edit connections
- Click Add
- Select Ethernet
- name the connection `shared-net`
- Under general, select Automatically connect when available and all users may
  use network
- Under IPv4 settings select Method: Shared to other computers

You should then be able to connect to the router using the settings setup on the
router at the beginning from the nuc on the robot

#### SSH-Keys, Hosts List, and SSHing in

You will now need to setup SSH Keys. Just follow
[this guide](https://www.digitalocean.com/community/tutorials/how-to-set-up-ssh-keys-on-ubuntu-1804)

You will probably also want to add the flo robot to your hosts list to make life easier.
To do that: edit your hosts file: `sudo nvim /etc/hosts` to have the nuc listed.
Add this: `10.42.0.189 flo-nuc`

You can also create a function in your bashrc that allows easier sshing in:

```bash
function ssh-flo {
    ssh nuc-admin@flo-nuc
}
```

### NUC

1. Setup Ubuntu
   1. Make sure to connect to a network and update everything
   2. Make sure to set the system to login automatically
   3. enable ssh: `sudo apt install openssh-server`
2. Use lsyncd with the configuration file (See [Developing](#developing)) to
   copy files over
3. ssh into the robot and run the install script (`./robot_install.sh`)
4. add a symlink to make running easier: ssh in and from the home directory type
   `ln -s ~/catkin_ws/src/LilFloSystem/robot_tmux_launcher.sh`.
5. You need to setup read/write privileges for all of the USB devices and setup
   fixed addresses using [udev](#udev)
6. The speaker may need to be [setup](#usb-speaker) a bit different
7. Settings and firmware will need to be updated on the
   [realsense cameras](#realsense-cameras)
8. You need to setup [Amazon Polly](#amazon-polly)
9. You will need to [setup the webrtc ros](#webrtc-ros) code
10. Add to the bashrc file on the robot:
    - `export ROBOT_NAME=<robots name>` The name should be the unique name of the
      robot. The current valid values are lilflo and mantaro, as we add more
      systems, each name must be unique
    - `export ROBOT_PASSWORD=<robot password>` This is the password generated by
      the webserver
    - `export FLO_SERVER_IP=< wherever the server is. Ex: "lilflo.com">` Which will allow the system to point
      to the webserver. During development you might have a different value here...
    - If you are working with a server that doesn't have real certs (this should
      only be true for development on a local machine). THen you also need to tell
      the router to not check certs by adding to the bashrc:
      `export NODE_TLS_REJECT_UNAUTHORIZED='0'`
11. Add two cron jobs to automatically startup the system:
    1. `crontab -e`
    2. `SHELL=/bin/bash` This will set the shell that things should run in
    3. `@reboot (sleep 90; source ~/.bashrc; ~/catkin_ws/src/LilFloSystem/robot_tmux_launcher.sh)`
    4. `*/1 * * * * (source ~/.bashrc; python ~/catkin_ws/src/LilFloSystem/flo_web/pinger/pinger.py)`

#### Assigning the serial devices to have a fixed addresses {#udev}

Setup
[UDEV Rules to make the face always have the same name](https://unix.stackexchange.com/a/183492):

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

If this is working correctly then the mode should be 666 to allow read/write
privileges. If it isn't working and getting a no access error, then you will
need to run `chmod 666 /dev/XXXX`. Then figure out what is wrong and fix it...

#### USB Speaker

I am not sure if this is needed:

You may need to make the usb speaker the default device: sudo vim
/usr/share/alsa/alsa.conf

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

The realsense cameras seem to hold a lot of settings on board. For now, you need
to plugin the camera and run `realsense-viewer`. For now, set the system to high
accuracy with the laser projector set to laser.

### Raspberry Pi

A Raspberry Pi cannot be used for most of this system. And it is not needed when
a beffier computer like a NUC is around. However, for running basic teleop a R/Pi
can be used. The install instructions are the same as the NUC, except you will
need to install ubuntu mate for raspberry pi armhf version. The realsense stuff
can't be installed, so be sure to comment that out of `gen_install.sh` before
you run it.

After installation, setting up openssh is a bit harder, update the system, install
openssh, then reconfigure it with sudo dpkg-reconfigure openssh-server

If you want to use the raspberry Pi camera, then run `sudo raspi-reconfigure`
and enable the camera. Then clone the [raspberry pi camera ros](https://github.com/UbiquityRobotics/raspicam_node)
repo into the `catkin_ws/src` repo and run:

```bash
sudo apt install libraspberrypi-dev libraspberrypi0
rosdep update
rosdep install --from-paths ~/catkin_ws/src/raspicam_node/ --ignore-src -r -y
cd ~/catkin_ws
catkin_make
```

### Amazon Polly

Polly will need to have access to AWS to work. To set that up, go to the
[IAM Console](https://console.aws.amazon.com/iam) and click on users. Then
select the user you want to give access to. Click "Create access key" under
Security Credentials. Then in the console on the computer run `./aws_install.sh`
which will
[install the AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-install.html)
for you and run `aws configure --profile flo`, it will prompt you for input an
you should fill in the info from the IAM console with region set to us-east-1
and output set to json.

A few notes:

- you should probably restrict what the user that is getting access can do. The
  best option is to set the permissions to AmazonPollyReadOnlyAccess
- You can have other users configured. By changing the --profile, you control
  the name of the user if no profile is specified, then the information is saved
  to a default
- If you ever need to revoke permissions for a user, that can be done from the
  IAM Console by deactivating the user's Access Keys.
- TODO: I would like to find a way to limit how many calls can be made by a
  user.
- To check whether polly is available given the profile, run
  `aws polly help --profile flo`. If you get back a list of commands to run on
  polly, you should be good.
- To fully test, can run:
  `aws polly synthesize-speech --output-format mp3 --voice-id Ivy --text 'hello, this is a test' --profile flo test.mp3`
  and then play the audio by installing: `sudo apt install mpg123` and typing
  `mpg123 test.mp3`
- you might find that you are getting some sort of server connection errors. you
  can resolve that by running `pip3 install -U boto3` (this should now be a part
  of the install script, but it is commented out)

### Getting WebRTC ROS installed {#webrtc-ros}

This will need to happen wherever you are running the webserver:

1. Clone the dev branch either into the catkin_ws/src folder or somewhere else
   and link it in for the repo: https://github.com/RobotWebTools/webrtc_ros
2. Add a file at `webrtc_ros/webrtc` named `CATKIN_IGNORE`, leave it empty
3. Build the catkin workspace as normal
4. make sure that at `LilFloSystem/flo_web/web_app/public` you link (`ln -s`) to
   `~/catkin_ws/src/webrtc_ros/webrtc_ros/web`. This should already be present.

The prior instructions give you the latest version of the package. This is
needed as the build farm version is not being updated quickly. However if
eventually the build farm version is used (likely via a rosdep dependency), then
the link will need to be changed to point to
`/opt/ros/<distro name>/share/webrtc_ros/web`

## Developing

When developing, all code changes should be made on the development computer.
Nothing should be getting changed on the robot. Instead code changes should be
made on the development computer and synced over. This allows multiple users to
work with the robot without login issues or anything of that nature while
keeping everything under source control.

To achieve this, we use lsyncd, which automatically pushes code changes to the
robot when running. Take a look at `templateSyncConfig.lua` to learn how to sync
your work over to the robot.

### Simulator

If you are developing on your local computer without the robot, there is a
simulator stack that you can use. under the `flo_core` package:
`mon launch flo_core flo_sim.launch`. You will also need to launch the webserver
by going to `flo_web/web_app` and running `npm start`. For development, you may
want to run the browser on your local machine with `npm chrome`

## Running

1. ssh into the robot: `ssh nuc-admin@<ip addr>`
2. Run the tmux launch script: `./robot_tmux_launcher.sh`
3. Attach to tmux: `tmux a`

When in tmux, you navigate between windows with `ctrl+b` + `w` + arrow keys. You
navigate between panes with `ctrl+b` + arrow keys. To leave tmux, just
progressively close programs with `ctrl+c` and type `exit` and they you can
close them as you go.

You can then launch your web browser and navigate to <http://10.42.0.189:3000/>
to see the web interface. You can also always use rviz and the other ROS tools
to make this all work on your local computer.

## Some useful tools:

For monitoring the kobuki, you can use the
[kobuki dashboard](http://wiki.ros.org/turtlebot_bringup/Tutorials/indigo/PC%20Bringup).
Run: `rqt -s kobuki_dashboard`.

## Getting Data off the Robot:

Once the robot has been used, you will need to get data off of it. There are a
few options for this:

- Transfer to external media, this is easy and fast, SSH into the robot,
  plug storage into the front USB3 ports, and use the `mv`
  (move) or `cp` (copy) commands to move things to the usb drive. You should be
  able to find your USB drive in the `/media/nuc-admin` folder. The cp command
  does not provide feedback, an alternative is to use rsync:
  `rsync -ah --progress <source> <destination>`
- Use SCP, this is a pain, not at all worth it
- Use an FTP gui, this works quite well but is limited by network speed, in your
  FTP gui, set the address as `ftp://flo-nuc` and then copy the data over the
  network.

All of the data should be stored in the `/home/nuc-admin/flo_data` folder.
There will be two types of files, rosbag files, 1 per minute of operation,
and parameter dumps, one from startup and one from shutdown.

## Things that may break: {#break}

### The web interface just doesn't do anything {#broken-web}

check to see if the socket is just closing imediately. If it is, then it may be
that the version of tornado on the server is wrong. Uninstall the version from
pip, then uninstall the version from apt, then reinstall the rosbridge suite.

### Packages are missing {#missing-packages}

It probably means you added dependencies since installing. Most dependencies are
installed via rosdep by running:

```bash
sudo apt update -y && sudo apt upgrade -y
rosdep update
rosdep install --from-paths ~/catkin_ws/src/LilFloSystem/ --ignore-src -r -y
```

Some dependencies can't be installed by rosdep and should be installed by the
install scripts. In that case, you should look at the most recent commits into
git for the install scripts and try to figure out which commands need to be run.
You could also just try to re-run the install scripts.

You might also be missing packages missing in the web server. Just navigate to
the web app folder (`flo_web/web_app`) and run `npm install`.

### Weird stuff says services don't exist {#broken-services}

Try running catkin_make on the catkin ws. If that doesn't work well, also delete
the install and dev folders in `catkin_ws` and run `catkin_make`

### REALSENSE has version mismatches and stuff {#broken-realsense}

apt update and upgrade Go into `catkin_ws/src/realsense-ros` run:

```bash
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
```

cd up to `catkin_ws` and `run catkin_make` followed by `catkin_make install`

It is also possible that you need to update the camera's firmware. To do this,
open realsense-viewer with the camera connected to a computer with an available
GUI. You will be prompted to update the firmware.

### Stuff just won't build {#broken-build}

All kinds of things could cause this. The first thing to try it to clean the
build and reinstall it:

```bash
cd ~/catkin_ws
catkin_make clean
catkin_make
catkin_make install
```

### There is no audio playing {#broken-audio}

The first thing to do is test the speaker by running `speaker-test` Then hop
into Alsa Mixer `alsamixer`, press F6 and select the USB Device, but this can
only change the volume. It is also possible that the wrong audio device is
selected. To fix that, you will need to select the audio device. List the
available devics by: `pacmd list-sinks|grep index` and then set the one you want
with `pacmd set-default-sink <sink_name|index>` You can also set the volume
here.

### Videos don't show up when running {#broken-web-video}

There are a few possible problems:

1. things only seem to work in chrome for now, so use that.
2. You may not have your devices enabled because the source is non-secure. Here
   is what to do: a. goto:
   chrome://flags/#unsafely-treat-insecure-origin-as-secure b. fill in with:
   `http://10.42.0.189:3000,http://10.42.0.189:9090,http://10.42.0.189:9091` c.
   change to enabled
