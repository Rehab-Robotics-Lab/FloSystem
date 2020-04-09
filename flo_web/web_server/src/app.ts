// lib/app.ts
import express from 'express';
import http from 'http';
import WebSocket from 'ws';
import url from 'url';
import { v4 as uuidv4 } from 'uuid';
//import bcrypt from "bcrypt";
//import passport from "passport";
//import session from "express-session";

const socketPort = 8080;
const expressPort = 3008;

// Create a new express application instance
const app: express.Application = express();

app.get('/', function (req, res) {
    res.send('Hello World!!!!');
});

app.listen(expressPort, '0.0.0.0', function () {
    console.log('Example app listening on port ' + expressPort);
});

//const server = https.createServer({
//cert: fs.readFileSync('/path/to/cert.pem'),
//key: fs.readFileSync('/path/to/key.pem')
//});
const server = http.createServer();

const robotWSserver = new WebSocket.Server({ noServer: true });
const clientWSserver = new WebSocket.Server({ noServer: true });

interface Robot {
    socket: WebSocket;
    rtcSocket: WebSocket | undefined;
    connected: string;
    name: string;
}

interface Client {
    socket: WebSocket;
    rtcSockets: Record<string, WebSocket>;
    name: string;
    connected: string;
}

const robots: Record<string, Robot> = {};
const clients: Record<string, Client> = {};

const closeClient = (client: string): void => {
    if (!(client in clients)) {
        console.error('tried to remove non-existant client');
        return;
    }
    const thisClient = clients[client];

    if (thisClient.connected in robots) {
        // If a client detaches, we will detach the robot and let it reconnect
        // Need to remove the connected string on the client to rpevent recursion
        robots[thisClient.connected].connected = '';
        closeRobot(thisClient.connected);
    }
    thisClient.socket.close();
    for (const key in thisClient.rtcSockets) {
        thisClient.rtcSockets[key].close();
    }
    delete clients[client];
};

const closeRobot = (client: string): void => {
    if (!(client in robots)) {
        console.error('tried to remove a non-existant robot');
        return;
    }
    const thisClient = robots[client];

    if (thisClient.connected in clients) {
        // If a robot disapears, we can't keep the client around
        clients[thisClient.connected].connected = '';
        closeClient(thisClient.connected);
    }
    thisClient.socket.close();
    if (thisClient.rtcSocket) {
        thisClient.rtcSocket.close();
    }
    delete robots[client];
};

// For connections from robots
robotWSserver.on(
    'connection',
    (ws: WebSocket, request: http.IncomingMessage, client: string) => {
        // check if webrtc connection, if so do something different
        console.log('Robot ' + client + ' connected');
        if (client in robots) {
            closeRobot(client);
        }
        robots[client] = {
            socket: ws,
            connected: '',
            name: client,
            rtcSocket: undefined,
        };
        ws.on('message', (msg) => {
            if (robots[client].connected in clients) {
                clients[robots[client].connected].socket.send(msg);
            }
        });
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
            const name = uuidv4();
            clients[client].rtcSockets[name] = ws;
            ws.on('message', (msg: string) => {
                //TODO: There are race conditions all over like here, the socket could close between the check and the sending
                const robotSock = robots[clients[client].connected].rtcSocket;
                if (typeof robotSock === 'undefined') {
                    console.error('webrtc socket to robot is broken');
                    return;
                }
                robotSock.send(JSON.stringify({ target: name, msg: msg }));
            });

            ws.on('close', () => {
                const robotSock = robots[clients[client].connected].rtcSocket;
                if (typeof robotSock === 'undefined') {
                    console.error('webrtc socket to robot is broken');
                    return;
                }
                robotSock.send(
                    JSON.stringify({ target: name, command: 'close' }),
                );
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
