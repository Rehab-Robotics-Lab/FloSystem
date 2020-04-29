# Web Controller for Flo Project

This is the web control code for the flo project.
There are a few different parts of this.

We are not going to assume any particulary hosting partner, IE AWS/Azure/Linode. 
We will be setting up as if we are self hosting, then it should be easy enough to push out to an offsite host if we decide to.
Deploying on prem is better for security, cost, and latency.

## Coding style
Everything uses typescript to wrap ecma script. 

## Security Vulnerabilities
As I go I am trying to make this secure. 
However, there are some shortcuts I need to take that pose big security holes.
The largest is that I do not validate anything that the robot or operator sends to the server.
It all just goes on through. 
This is very bad.
This risk is mitigated by only allowing authorized users in. 
Eventually all of the communications in both directions should be sanitized and the entire roslib stack should be replaced with something better. 

## ROS Code
On the ros side, `src/ws_client.py` connects the local robot to the remote webserver using websockets. 
This uses the robot web tools ?? API
One of the things that this does is handle multiple clients over a single connection.
This is because we are opening the one connection to the server, but multiple clients on the other side might want to connect and publishing of messages out needs to specify which client it is going to.

The ROS side code is launched using `launch/web_app.launch`.

## Server
The server is NodeJS running express sitting behind Nginx.
Nginx handles static file serving.
[PM2](https://pm2.keymetrics.io/) is used to keep the Node server up. 
SystemD is used to start Nginx and PM2

### Node
Node is running a lot of stuff.
A few notes:
- Set `NODE_ENV=production` <- env variable. To optimize a bunch of stuff

### SSL
The ssl and encryption are all handled by Nginx. 
Instructions for setting up generically are here: https://www.sitepoint.com/configuring-nginx-ssl-node-js/ 
We are using a cert from namecheap.
Instructions for NameCheap certs are here: 

### Gzip
Compression is handled by NGinx

### Ports
- 80: Nginx

### Socket Routing
The core of the system is routing connections from the robot, through a server, to an operator. 
The robot is expected to connect to the server when it turns on
Nginx needs to [be configured](https://www.tutorialspoint.com/how-to-configure-nginx-as-reverse-proxy-for-websocket) to properly proxy websockets.

Initially, we will only run one node server, so routing can be in the server and it can be a little stateful. 
Eventually we may want a [pub/sub model](https://redis.io/topics/pubsub) of some sort or other internal communication to run on multiple servers at once and handle fail over. 
Eventually messages should be buffered to handle connection drops? probably by redis. 

### Proxy
Nginx is used for proxying in front of node

#### Web Sockets

### Static file hosting
Nginx is used to serve static files

### Authentication
Authentication is done with [passport]().
Password hashing is done with [bcrypt](https://www.npmjs.com/package/bcrypt)

### Database 

#### Users/Robots
The users and robots database is in mysql

### TypeScript
For production, run `npm run tsc` to compile the server into the build folder. 
For development, run `npm run dev`. 

#### Robot Control Page
Instructions for hosting a create react app based app on Nginx: https://medium.com/@timmykko/deploying-create-react-app-with-nginx-and-ubuntu-e6fe83c5e9e7

## Client/Operator

### Robot control page
The operator code is all written in react using create-react-app in `web_app`. 

### Login Page


## STUN and TURN servers
For fully robust WebRTC, we need to have STUN servers to get the routing details and TURN survers to fall back on if the peer to peer connection fails.
I think that Jitsi is the best way to provide all of this. 
