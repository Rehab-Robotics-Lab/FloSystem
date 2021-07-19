# LilFloSystem

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

*   [WebServer Setup](#webserver-setup)
*   [Development Computer Setup](#dev-computer)
*   [Network Setup](#network)
*   [NUC Setup](#nuc)
    *   [UDEV for USB fixed Addresses](#udev)
    *   [USB Speker](#usb-speaker)
    *   [Realsense Cameras](#realsense-cameras)
*   [Amazon Polly Setup](#amazon-polly)
*   [WebRTC ROS Setup](#webrtc-ros)
*   [Developing on The System](#developing)
    *   [Simulator](#simulator)
*   [Running Code](#running)
*   [Things That May Break](#break)
    *   [The Web Interface Doesn't Respond](#broken-web)
    *   [Packages are missing](#missing-packages)
    *   [Services Don't Exist](#broken-services)
    *   [Realsense has version mismatches](#broken-realsense)
    *   [ROS won't build](#broken-build)
    *   [No audio plays](#broken-audio)
    *   [No videos on web](#broken-web-video)
*   \[Camera Recalibration]
    *   \[Practical Tips]
    *   \[Realsense Tools]
    *   \[OpenCV]

## WebServer Setup

Setting up the webserver is a totally different issue from setting up the robot.
Everything runs in docker. Let's walk through real quickly what is going on:
When anything outside hits the webserver stack, they are going to route through
Nginx, which is listening on both ports 80 and 443. The definition for Nginx
depends on the environment, production or development, and is
defined in `flo_web/nginx-prod.conf` and `flo_web/nginx-dev.conf`
respectively. In production, Nginx will serve the static front end
files itself. In development create-react-app (via nodejs) will serve
them so that the developer has hot reloading. The Nginx docker image
is defined in `flo_web/Dockerfile` where the front end is first compiled
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

### Server Config Files

You should create a file in `LilFloSystem/certs/session-secret.env` 
with contents: `SESSION_SECRET=<cryptographically random value ex:random string of characters or numbers>`

You should create a file in `LilFloSystem/certs/coturn.env`
with contents:

```conf
COTURN_SECRET=<cryptographically random value>
SITE_ADDR=<the site address, ex: lilflo.com>
```

You should create a file in `LilFloSystem/certs/datadog.env`
with contents:

```conf
DD_API_KEY=<cryptographically random value>
DD_HOSTNAME=<hostname ex: lilflo.com>
```

### SSL certs

#### Local

We need ssl certs during local development.

1.  `sudo apt install libnss3-tools`
2.  Download the latest binary for [mkcert](https://github.com/FiloSottile/mkcert) Then install mkcert using installation guide on the github
3.  Change permissions `sudo chmod u+x <name of binary ex:mkcert>`
4.  Setup certs registry: `<name of binary: mkcert...> -install`
5.  Make a certs dir/folder (named `certs`) in the root of this repo and go into it
6.  Make new certs: `<name of binary: mkcert...> localhost` you can add other options here if you want to simulate a local domain by putting it in your hosts file, add in the 127.0.0.1 or other localhost aliases, etc. You can even use wildcards. NOTE: be very careful with these, they can really really open you up to security holes in your local computer if shared.
7.  rename the certs: `mv localhost-key.pem localhost.key && mv localhost.pem localhost.crt` Of course if your certs are named something else... you get the idea.

#### Server

We use certbot to generate certificates automatically. The certbot certificates
automatically. They expire every so often, so they need to be regularly regenerated.
We have it run once a week on sunday at 2:30AM eastern to regen certs and
3:30AM eastern to restart the server and use the new certs.
The whole system runs based on a [guide](https://medium.com/@pentacent/nginx-and-lets-encrypt-with-docker-in-less-than-5-minutes-b4b8a60d3a71)

### Setting up first admin

When you initially create the system, you will have no users. You can register a new
user, but that user will not have administrator privelages. You need to setup a first
admin. To do this:

1.  Create a user account by registering through the web interface
2.  Attach to the docker image for the postgres database into pg: `docker container exec -it lilflosystem_postgres_1 psql flodb -h localhost -U postgres` (note the number on that container might be different...)
3.  Turn on expanded view to get nice prints: `\x`
4.  Optionally get info on your registered users: `select * from users;`
5.  Find out what id admin users need: `select * from user_types;`
6.  Set the user you are interested in to be an admin: `update users set user_type=<id of usertype you want> where email=<email you want>;`. Ex: `update users set user_type=1 where email='testsobrep@seas.upenn.edu';`
7.  Check that it worked: `select * from users;`

### Deploying to Linode

Linode is small, easy to use, and affordable.

1.  Setup a node on linode. The small size should be fine. When setting up, setup an ssh-key to make your life easier. Use the latest Ubuntu LTS.
2.  Setup the A/AAA record to work with your domain name and have your register use the linode domain servers.
3.  Install and setup firewall:
    1.  `apt install ufw`
    2.  `ufw default allow outgoing`
    3.  `ufw default deny incoming`
    4.  `ufw allow ssh`
    5.  `ufw allow http`
    6.  `ufw allow https`
    7.  `ufw enable`
    8.  Check status with `ufw status`
4.  [Setup unattended updates](https://help.ubuntu.com/lts/serverguide/automatic-updates.html): `apt install unattended-upgrades`
5.  Clone this repository
6.  update and upgrade: `apt update -y && apt upgrade -y`
7.  [Install docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
8.  Install docker-compose: `apt install docker-compose`
9.  Go into the repo root and run `docker-compose -f docker-compose.yml -f docker-compose.prod.yml build`
10. Run `docker pull certbot/certbot`
11. Setup certificates by running `./init-letsencrypt.sh`
12. Run `docker-compose -f docker-compose.yml -f docker-compose.prod.yml up -d`
13. Monitor with `docker ps`, `docker stats`, and `docker-compose logs -f`
14. If you need to take it down, run: `docker-compose down`

### Deploying TURN Server

In some network situations, you will need a turn server. In those cases,
you probably want to be running UDP, TCP, and TLS on port 443 to allow
packets through.

After trying to get this to work behind Nginx to get this all on one server,
I have concluded that the turn server should be on an independent vm. This
makes the port management easier and prevents the turn server from taking
resources from the rest of the system.

1.  Create a new Ubuntu 18 vm.
2.  Create an A/AAA record for `turn.<url>`
3.  Install and setup firewall:
    1.  `apt install ufw`
    2.  `ufw default allow outgoing`
    3.  `ufw default deny incoming`
    4.  `ufw allow ssh`
    5.  `ufw allow http`
    6.  `ufw allow https`
    7.  `ufw allow 80/udp`
    8.  `ufw allow 443/udp`
    9.  `ufw allow 49152:65535/udp`
    10. `ufw allow 49152:65535/tcp`
    11. `ufw enable`
    12. Check status with `ufw status`
4.  [Setup unattended updates](https://help.ubuntu.com/lts/serverguide/automatic-updates.html): `apt install unattended-upgrades`
5.  Clone this repo
6.  update and upgrade: `apt update -y && apt upgrade -y`
7.  [Install docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
8.  Install docker-compose: `apt install docker-compose`
9.  Go into the repo root and run `docker-compose -f docker-compose-turn.yml build`
10. Run `docker pull certbot/certbot`
11. Setup certificates by running `./init-letsencrypt-turn.sh`
12. Run `docker-compose -f docker-compose-turn.yml up -d`
13. Monitor with `docker ps`, `docker stats`, and `docker-compose logs -f`

## Robot

Throughout this setup, we will assume that you are using a clean install of the
version of linux that is called for. The setup may not tolerate weird tweaks to
the python version, will not work with conda, and will not work with python
virtual environments.

The first think to do is to setup your development computer:

### Dev Computer

You will need to have a development and control computer. At the least you will
need to follow the instructions in the [Network](#network) section. If you want
to actually be writing new code and devloping, you will need all of this:

1.  Setup Ubuntu, either 16 or 18
2.  Install git: `sudo apt intall git-all`
3.  Clone this repo into `~/Documents/git`
4.  Run `./dev_install.sh`
5.  Setup your bashrc or zshrc to have ros stuff. There an
    [example loader](example-loader) that you can take a look at.
6.  If you are going to be testing the text to speech system on your development
    computer, then you need to follow the
    [Amazon Polly Instructions](#amazon-polly)
7.  Node, is needed to develop and run the websever. You can install directly
    (better for the robot) using `node_install.sh` or with nvm (better for
    development) using `nvm_install.sh`. If you are working with the web server,
    you will also need to install
8.  You will need to [setup the network](#network)

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

1.  all of the necessary ROS files will be sourced
2.  based on the computer you are on, the correct NIC will be used to get the IP
    address.
    1.  Note that for mjs-mws, there are two NICs of interest, that are ordered by
        priority. This allows us to use a wired network if available and if not to
        fall back to the wireless.
    2.  If there is no IP address, ie. your not connected to a network, then the
        loopback device will be used.
3.  Based on the input given, the ros master will be set to connect to a specific
    robot.
    1.  For zero, the local machine will be connected to.
    2.  Other options are `flo`, `kq`, `x220t`. You probably don't need all of
        these, but it gives you a nice template of how to work.

Some things to keep in mind:

*   You need to adjust the hostnames section. You should use the host name of your
    computer and the network adapter you want. You can find the names of your
    network adapter by typing either `ifconfig` or `ip a` in the terminal
*   You will likely need to add a different robot. You can simply use the ones
    that are above as a template. You will need your common language name for the
    robot and its IP address.
*   `connect_to_robot 0` will connect to your local robot

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

1.  Setup Ubuntu
    1.  Make sure to connect to a network and update everything
        *   When connecting to the network, you should obscure your password. To do this, run wpa_passphrase \[ssid-name]\[password-name] and use the result for the network. Set the password to only save for this user (little logo at the end of the password line)
    2.  Make sure to set the system to login automatically
    3.  enable ssh: `sudo apt install openssh-server`
2.  Use lsyncd with the configuration file (See [Developing](#developing)) to
    copy files over
3.  ssh into the robot and run the install script (`bash ./robot_install.sh`)
4.  add a symlink to make running easier: ssh in and from the home directory type
    `ln -s ~/catkin_ws/src/LilFloSystem/robot_tmux_launcher.sh`.
5.  You need to setup read/write privileges for all of the USB devices and setup
    fixed addresses using [udev](#udev)
6.  The speaker may need to be [setup](#usb-speaker) a bit different
7.  Settings and firmware will need to be updated on the
    [realsense cameras](#realsense-cameras)
8.  You need to setup [Amazon Polly](#amazon-polly)
9.  You will need to [setup the webrtc ros](#webrtc-ros) code
10. Add to the bashrc file on the robot:
    *   `export ROBOT_NAME=<robots name>` The name should be the unique name of the
        robot. The current valid values are lilflo and mantaro, as we add more
        systems, each name must be unique
    *   `export ROBOT_PASSWORD=<robot password>` This is the password generated by
        the webserver
    *   `export FLO_SERVER_IP=< wherever the server is. Ex: "lilflo.com">` Which will allow the system to point
        to the webserver. During development you might have a different value here...
    *   If you are working with a server that doesn't have real certs (this should
        only be true for development on a local machine). THen you also need to tell
        the router to not check certs by adding to the bashrc:
        `export NODE_TLS_REJECT_UNAUTHORIZED='0'`
11. Add two cron jobs to automatically startup the system:
    1.  `crontab -e`
    2.  `SHELL=/bin/bash` This will set the shell that things should run in
    3.  `@reboot (sleep 90; source ~/.bashrc; ~/catkin_ws/src/LilFloSystem/robot_tmux_launcher.sh)`
    4.  `*/1 * * * * (source ~/.bashrc; python ~/catkin_ws/src/LilFloSystem/flo_web/pinger/pinger.py)`
12. Setup firewall (really need that with ros)
    1.  `sudo ufw default allow outgoing`
    2.  `sudo ufw default deny incoming`
    3.  `sudo ufw allow ssh`
    4.  `sudo ufw enable`
    5.  check: `sudo ufw status`
13. Take a look at the bash_includes file. It should be going in through the install system automatically. Might not be though. For testing you want to set the ros_ip. But for deployment you do not. If a ROS IP is set using a network which the robot is connected to, then upon network loss the ros system will crash

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

*   you should probably restrict what the user that is getting access can do. The
    best option is to set the permissions to AmazonPollyReadOnlyAccess
*   You can have other users configured. By changing the --profile, you control
    the name of the user if no profile is specified, then the information is saved
    to a default
*   If you ever need to revoke permissions for a user, that can be done from the
    IAM Console by deactivating the user's Access Keys.
*   TODO: I would like to find a way to limit how many calls can be made by a
    user.
*   To check whether polly is available given the profile, run
    `aws polly help --profile flo`. If you get back a list of commands to run on
    polly, you should be good.
*   To fully test, can run:
    `aws polly synthesize-speech --output-format mp3 --voice-id Ivy --text 'hello, this is a test' --profile flo test.mp3`
    and then play the audio by installing: `sudo apt install mpg123` and typing
    `mpg123 test.mp3`
*   you might find that you are getting some sort of server connection errors. you
    can resolve that by running `pip3 install -U boto3` (this should now be a part
    of the install script, but it is commented out)

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

For developing anywhere, the best thing to do is to use the docker stack of
simulators.

If you are on a computer with the realsense cameras plugged in and you can install
the entire stack locally, then you can run `sim_tmux_launcher.sh`. This will run
the entire Lil'Flo system in simulation and run the web servers for you. For this
to work, you will need to [setup the config files and keys needed by the webserver
](#WebServer-Setup)

If you are on a machine that does not have the entire stack installed, then you can
run an entire simulation stack within docker. To do this:

1.  Follow the instructions for [setting up ssl certs](#ssl-certs)

2.  Follow instructions for [setting up config files](#config-files)

3.  Install [docker](https://docs.docker.com/desktop/) and complete the post installation steps, then test docker by running the hello world image

4.  Run `docker-compose up`

5.  Open Chrome, and navigate to localhost

6.  Follow instructions to [setup admin](#setting-up-first-admin)

7.  Login with your new admin account, go into the admin portal, click add robot,
    give the robot a name and type of lilflo and before submiting copy the password to clipboard and save it. Then hit submit.

8.  Make a file `./certs/sim-info.env` with `ROBOT_NAME=<name from web interfave>`
    and `ROBOT_PASSWORD=<password from web interface>`

9.  Make a file `./certs/aws-credentials` and populate it with a valid AWS credential
    which has [access to aws polly](#amazon-poly). The first line should have
    `[flo]` the second line should have `aws_access_key_id = <the access key id>`
    the third line should have `aws_secret_access_key = <secret key>`. You also need
    to create `./certs/aws-config` and pupulate that with valid AWS config. First
    line should be `[profile flo]` second line should be the region, ex:
    `region = us-east-1` third line should be the output `output = json`

10. Run `./docker_sim_launcher.sh`. This file can be passed `-r` to rebuild the
    underlying docker images for the robot if you have changed code

11. Go to localhost

12. To inspect the system, in an unused terminal:
    `docker exec -it <container name, ex: lilflosystem_flo_sim_1> /ros_entrypoint.sh bash`

13.  After entering localhost Add the newly created robot to your profile under admin portal.

Note: This will mount your local code, so you don't have to shut the entire system down for every
code change. For the web frontend, code will reload automatically on save (sometimes you need to
save twice). For python files running on the robot, you can simply kill the affected node(s) using
your docker exec access terminal (described above) and when they restart they will have your new code

## Running

If fully set up as described above, then when the robot powers on,
it will automatically start all of the software after a short delay,
connect to the server, and be ready to use. If that has not been
done, then:

1.  ssh into the robot: `ssh nuc-admin@<ip addr>`
2.  Run the tmux launch script: `./robot_tmux_launcher.sh`
3.  Attach to tmux: `tmux a`

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

*   Transfer to external media, this is easy and fast, SSH into the robot,
    plug storage into the front USB3 ports, and use the `mv`
    (move) or `cp` (copy) commands to move things to the usb drive. You should be
    able to find your USB drive in the `/media/nuc-admin` folder. The cp command
    does not provide feedback, an alternative is to use rsync:
    `rsync -ah --progress <source> <destination>`
*   Use SCP, this is a pain, not at all worth it
*   Use an FTP gui, this works quite well but is limited by network speed, in your
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

PAUSE!! Ok, Realsense is a pain. Intel really doesn't want us using
their hardware. The first thing to do is to go check the latest
releases from realsense-ros. What version of librealsense is supported?
You probably want to update to the latest version of realsense ros
but only want to upgrade to the version of librealsense that that
supports (wtf intel?). Good luck my friend

apt update and upgrade Go into `catkin_ws/src/realsense-ros` run:

```bash
git fetch
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
```

cd up to `catkin_ws` and run:

```bash
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
```

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

1.  things only seem to work in chrome for now, so use that.
2.  You may not have your devices enabled because the source is non-secure. Here
    is what to do: a. goto:
    chrome://flags/#unsafely-treat-insecure-origin-as-secure b. fill in with:
    `http://10.42.0.189:3000,http://10.42.0.189:9090,http://10.42.0.189:9091` c.
    change to enabled

\#Recalibrating the Realsense

## Practical tips

1.  The Intel D415 has the following parameters:
    *   Intrinsic : Focal length, Distortion and Principal Point for each of the three cameras.
    *   Extrinsic : baseline, RotationLeftRight, TranslationLeftRight, RotationLeftColor, TranslationLeftColor

2.  The camera might be out of calibration if flat surfaces look noisy/wobbly, depth images have many holes, physical distances are not within 3% of the expected distance. This might happen if the camera falls down/the factory calibration changes due to some other event which in not very frequent.

\##Using Realsense Tools

1.  On chip self calibration can be performed using the Realsense Viewer(comes with Intel® RealSense™ SDK 2.0). This tool provides a health-check of current calibration. If the value is below 0.25, the calibration is good. Anything above 0.75 needs recalibration. The intrinsics and extrincs can be calibrated by pointing the camera at a flat white wall in good lighthing condition. The application also scores and allows comparison of new calibrations. For more: https://dev.intelrealsense.com/docs/self-calibration-for-depth-cameras

2.  Extrinsic calibration using Intel Realsense Dynamic Calibrator(https://dev.intelrealsense.com/docs/intel-realsensetm-d400-series-calibration-tools-user-guide). The guide provides instruction for downloading the Dynamic Calibrator app. Targets and Demos can also be found at the same link.

\##Using external tools

One way to check calibration is to print checkerboard targets(https://boofcv.org/index.php?title=Camera_Calibration_Targets). Then use: https://github.com/IRIM-Technology-Transition-Lab/camera-calibration , or equivalent to compare current parameter values with those returned from the calibration program.
