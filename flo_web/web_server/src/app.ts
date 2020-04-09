// lib/app.ts
import express from 'express';
import http from 'http';
import WebSocket from 'ws';
import url from 'url';
import { v4 as uuidv4 } from 'uuid';
//import bcrypt from "bcrypt";
//import passport from "passport";
//import session from "express-session";

const socketPort = 8080; // the port that the socker server is exposed on

const server = http.createServer(); // The server that will host our sockets

//Server for the robots to connect to:
const robotWSserver = new WebSocket.Server({ noServer: true });
//Server for the clients/operators to connect to:
const clientWSserver = new WebSocket.Server({ noServer: true });

interface Robot {
    // The socket that handles the roslibjs server traffic to the robot
    socket: WebSocket;
    // The socket that handles webrtc calls, going to a router on the robot
    rtcSocket: WebSocket | undefined;
    // The id of the client/operator that this robot is connected to, empty string if none.
    connected: string;
    // The name/id of this robot
    name: string;
}

interface Client {
    // The socket that handles the roslibjs traffic to the frontend interface
    socket: WebSocket;
    // The sockets which are connected from the front end to handle webrtc
    // calls:
    rtcSockets: Record<string, WebSocket>;
    // The name/id of this client:
    name: string;
    // Which robot this client is connected to:
    connected: string;
}

// The dictionary of all of the connected robots, indexed by their name
const robots: Record<string, Robot> = {};
// The dictionary of all of the connected front-end clients, indexed by their name
const clients: Record<string, Client> = {};

// When a client is being removed, we have to remove the references to it
// and close its sockets:
const closeClient = (client: string): void => {
    if (!(client in clients)) {
        console.error('tried to remove non-existant client');
        return;
    }
    const thisClient = clients[client];

    if (thisClient.connected in robots) {
        // If a client detaches, we will detach the robot and let it reconnect
        // Need to remove the connected string on the client to prevent recursion
        robots[thisClient.connected].connected = '';
        closeRobot(thisClient.connected);
    }
    thisClient.socket.close();
    for (const key in thisClient.rtcSockets) {
        thisClient.rtcSockets[key].close();
    }
    delete clients[client];
};

// When a robot is being removed, we need to detach its clients and clean
// up its sockets:
const closeRobot = (robot: string): void => {
    if (!(robot in robots)) {
        console.error('tried to remove a non-existant robot');
        return;
    }
    const thisRobot = robots[robot];

    if (thisRobot.connected in clients) {
        // If a robot disapears, we can't keep the client around
        // We remove the reference back here to prevent recursion
        clients[thisRobot.connected].connected = '';
        closeClient(thisRobot.connected);
    }
    thisRobot.socket.close();
    if (thisRobot.rtcSocket) {
        thisRobot.rtcSocket.close();
    }
    delete robots[robot];
};

// For connections from robots
robotWSserver.on(
    'connection',
    (ws: WebSocket, request: http.IncomingMessage, robot: string) => {
        if (request.url === undefined) {
            ws.close();
            return;
        }
        const path = url.parse(request.url).pathname;
        if (path === null) {
            ws.close();
            return;
        }
        const pathSplits = path.split('/');
        if (pathSplits.length < 2) {
            ws.close();
            return;
        }

        // check if webrtc connection, if so do something different
        if (pathSplits.length >= 2 && pathSplits[2] === 'webrtc') {
            if (!(robot in robots)) {
                console.error(
                    'Robot tried to connect webrtc channel before api channel',
                );
                return;
            }
            console.log('Robot ' + robot + ' conected a new webrtc channel');
            const sendToClient = (
                target: string,
                command: string,
                msg: string,
            ) => {
                if (!(target in clients[robots[robot].connected].rtcSockets)) {
                    console.error(
                        'message from robot for a non-existant webrtc connection path',
                    );
                    return;
                }
                const clientSock =
                    clients[robots[robot].connected].rtcSockets[target];
                //TODO: There are race conditions all over like here, the socket could close between the check and the sending
                if (typeof clientSock === 'undefined') {
                    console.error('webrtc socket to robot is broken');
                    return;
                }

                clientSock.send(msg);
            };
        } else {
            console.log('Robot ' + robot + ' connected');
            if (robot in robots) {
                closeRobot(robot);
            }
            robots[robot] = {
                socket: ws,
                connected: '',
                name: robot,
                rtcSocket: undefined,
            };
            ws.on('message', (msg) => {
                if (robots[robot].connected in clients) {
                    clients[robots[robot].connected].socket.send(msg);
                }
            });
        }
    },
);

robotWSserver.on('listening', () => {
    console.log('robot websocket server listening');
});

// For connections from clients (operators)
clientWSserver.on(
    'connection',
    (ws: WebSocket, request: http.IncomingMessage, client: string) => {
        if (request.url === undefined) {
            ws.close();
            return;
        }
        const path = url.parse(request.url).pathname;
        if (path === null) {
            ws.close();
            return;
        }
        const pathSplits = path.split('/');
        if (pathSplits.length < 3) {
            ws.close();
            return;
        }
        const targetRobot = pathSplits[2];

        if (!(targetRobot in robots)) {
            console.error('Non-existant target robot: ' + targetRobot);
            ws.close();
            return;
        }
        // check if webrtc connection, if so do something different
        if (pathSplits.length >= 3 && pathSplits[3] === 'webrtc') {
            // are we already connected to this robot?
            if (clients[client].connected !== targetRobot) {
                console.error(
                    'tried to connect a webrtc line to ' +
                        'a robot that is not bound to this operator',
                );
                ws.close();
                return;
            }
            const sendToRobot = (
                target: string,
                command: string,
                msg: string,
            ) => {
                const robotSock = robots[clients[client].connected].rtcSocket;
                //TODO: There are race conditions all over like here, the socket could close between the check and the sending
                if (typeof robotSock === 'undefined') {
                    console.error('webrtc socket to robot is broken');
                    return;
                }

                robotSock.send(
                    JSON.stringify({
                        target: target,
                        command: command,
                        msg: msg,
                    }),
                );
            };

            const name = uuidv4();
            clients[client].rtcSockets[name] = ws;
            sendToRobot(name, 'open', '');

            ws.on('message', (msg: string) => {
                sendToRobot(name, 'msg', msg);
            });

            ws.on('close', () => {
                sendToRobot(name, 'close', '');
            });
        } else {
            console.log(
                'Client ' + client + ' connected seeking robot: ' + targetRobot,
            );
            if (client in clients) {
                clients[client].socket.close();
            }
            if (robots[targetRobot].connected !== '') {
                console.error('Tried to connected to an already taken robot');
                ws.close();
                return;
            } else {
                //TODO: get rid of this else statement, the if terminates with a retun
                clients[client] = {
                    socket: ws,
                    connected: '',
                    name: client,
                    rtcSockets: {},
                };
                //TODO: this is a potential race condition
                robots[targetRobot].connected = client;
                clients[client].connected = targetRobot;
            }

            ws.on('message', (msg) => {
                robots[targetRobot].socket.send(msg);
            });
        }
    },
);

clientWSserver.on('listening', () => {
    console.log('robot websocket server listening');
});

server.on('upgrade', (request, socket, head) => {
    const path = url.parse(request.url).pathname;
    console.log('socket upgrade with path: ' + path);
    if (path === null) {
        console.error('null path');
        socket.close();
        return;
    }
    const pathSplits = path.split('/');
    if (pathSplits.length < 2) {
        console.error('not enough elements on path');
        socket.close();
        return;
    }
    const target = pathSplits[1];
    console.log('connection attempting to ' + target);

    if (target === 'host') {
        // For the robots to connect to
        robotWSserver.handleUpgrade(request, socket, head, (ws) => {
            robotWSserver.emit('connection', ws, request, 'flo');
        });
    } else if (target === 'robot') {
        // For the clients to hook up to a robot
        clientWSserver.handleUpgrade(request, socket, head, (ws) => {
            clientWSserver.emit('connection', ws, request, 'operator1');
        });
    } else {
        console.log('invalid websocket enpoint');
        socket.close();
    }
});

server.on('connect', (req: http.IncomingMessage, socket: any, head: Buffer) => {
    console.log('connect event');
});

server.on('listening', () => {
    const address = server.address();
    if (address === null) {
        console.error('something very wrong with starting server');
        return;
    }
    if (typeof address === 'string') {
        console.log('Socket server listening at: ' + address);
    } else {
        console.log(
            'Socket server listening at: ' +
                address.address +
                ':' +
                address.port,
        );
    }
});

server.listen(socketPort, '0.0.0.0');
