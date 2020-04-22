// lib/app.ts
import express from 'express';
import bodyParser from 'body-parser';
import http from 'http';
import WebSocket from 'ws';
import url from 'url';
import { v4 as uuidv4 } from 'uuid';
import net from 'net';
import pg from 'pg';
import bcrypt from 'bcrypt';
import mountRoutes from './routes';
import * as db from './db';

import redis from 'redis';
import session from 'express-session';
import connectRedis from 'connect-redis';
//import passport from "passport";
//import session from "express-session";

const apiPort = 3030;
const app = express();
const sessionSecret = process.env.sessionsecret || 'secret';

app.set('trust proxy', 1); // allows us to use nginx with secure cookie in sessions
// TODO: test if we need this.

const RedisStore = connectRedis(session);

const sessionStore = new RedisStore({
    client: redis.createClient({ host: 'session-store', port: 6379 }),
});

app.use(
    session({
        secret: sessionSecret,
        store: sessionStore,
        resave: false,
        saveUninitialized: false,
        cookie: {
            secure: true,
            maxAge: 1000 * 60 * 60 * 48, // 48 hours
        },
    }),
);

// Parse the string in the requests into json:
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: true }));
// Load in our routes:
mountRoutes(app);
// start server:
app.listen(apiPort, () => {
    console.log('API server running on port: ' + apiPort);
});

/**
 * A connection between the server and an external client (either an operator or the robot).
 *
 * @remarks
 * All client types  will have two communication channel types, a ros data socket and a (or set of)
 * webrtc data channels. The ros data channel acts as a relay for webrtc connection messages.
 *
 */
class Connection {
    socket: WebSocket;
    rtcSockets: Map<string, WebSocket>;
    connected: Connection | undefined; // the Name of the connected operator or empty string if none
    name: string; // The name of this robot

    constructor(ws: WebSocket, name: string) {
        this.socket = ws;
        this.name = name;
        this.rtcSockets = new Map();
    }

    async close() {
        this.socket.close();
        if (this.connected !== undefined) {
            this.connected.connected = undefined; //prevent recursion
            this.connected.close();
        }
        this.rtcSockets.forEach((sock) => {
            sock.close();
        });
        await db.query(
            'update robots set connected=false where robot_name=$1',
            [this.name],
        );
    }

    onMessage(msg: any) {
        if (this.connected !== undefined) {
            this.connected.socket.send(msg);
        }
    }

    ping() {
        if (this.connected !== undefined) {
            this.connected.socket.ping();
        }
    }

    pong() {
        if (this.connected !== undefined) {
            this.connected.socket.ping();
        }
    }
}

/**
 * Parse a URL to the websocket server
 *
 * @remarks
 *
 * This is made to take the URL from the http request and determine whether
 * the other side is a an operator looking for a robot or a robot connecting.
 *
 * URLs come in the following formats:
 * - lilflo.com/host                      : Robot connecting roslib
 * - lilflo.com/host/webrtc               : Robot connecting webrtc messaging
 * - lilflo.com/robot/<robot name>        : Operator connectint to robot for roslib
 * - lilflo.com/robot/<robot name>/webrtc : Operator looking to connect webrtc to robot
 *
 * @param reqUrl - The url to parse
 * @returns undefined if there was an error in the url or a dictionary with the
 *          originType (operator or robot), what the origin is trying to connect to
 *          (empty string if coming from robot, robot name if coming from operator),
 *          and webrtc, whether this is a webrtc connection
 */

function parseUrl(
    reqUrl: string | undefined,
):
    | undefined
    | { originType: 'robot' | 'operator'; target: string; webrtc: boolean } {
    if (reqUrl === undefined) {
        return undefined;
    }
    const path = url.parse(reqUrl).pathname;
    if (path === null) {
        return undefined;
    }
    const pathSplits = path.split('/');
    if (pathSplits.length < 2) {
        return undefined;
    }

    let webrtc = false;
    let target: string = '';
    let originType: 'robot' | 'operator';

    if (pathSplits[1] === 'robot') {
        originType = 'operator';
        if (pathSplits.length < 3) {
            return undefined;
        }
        target = pathSplits[2];
        if (pathSplits.length > 3 && pathSplits[3] === 'webrtc') {
            webrtc = true;
        }
    } else if (pathSplits[1] === 'host') {
        originType = 'robot';
        if (pathSplits.length > 2 && pathSplits[2] === 'webrtc') {
            webrtc = true;
        }
    } else {
        return undefined;
    }
    return { originType, target, webrtc };
}

class Connections {
    server: WebSocket.Server;
    clients: Map<string, Connection>;

    constructor() {
        this.server = new WebSocket.Server({ noServer: true });
        this.clients = new Map();
        this.server.on('connection', this.onConnection);
    }

    onConnection(
        ws: WebSocket,
        request: http.IncomingMessage,
        name: string,
        target: string,
        webrtc: boolean,
    ) {
        throw new Error('not implemented');
    }
}

/**
 * For all of the connections from the robots.
 *
 * @remarks
 * There are two kind of connections a robot can make: data and webrtc.
 */
class RobotConnections extends Connections {
    onConnection(
        ws: WebSocket,
        request: http.IncomingMessage,
        name: string,
        target: string,
        webrtc: boolean,
    ) {
        console.log(
            'new robot connection from: ' + name + ' webrtc: ' + webrtc,
        );
        if (webrtc) {
            this.onWebRtcConnection(ws, name);
        } else {
            this.onDataConnection(ws, name);
        }
    }

    /**
     * Handle a robot connecting its webrtc channel.
     *
     * @remarks
     * Connections coming here will be from the router on the robot.
     */
    onWebRtcConnection(ws: WebSocket, name: string) {
        const thisClient = this.clients.get(name);
        if (thisClient === undefined) {
            console.error('tried to connect to webrtc before data');
            ws.close();
            return;
        }

        // There will only be one webrtc socket from a robot that will carry
        // all of the data. Although there webrtc connections are stored in a
        // map, the only key which they will ever have is `robot`
        // TODO: should we be checking if there is already a socket attached here?
        //       (should not happen...)
        thisClient.rtcSockets.set('robot', ws);

        console.log('Robot ' + name + ' conected webrtc socket');

        /**
         * For sending to the operator connected to this socket
         *
         * @param id - The id of the connection the message should go to.
         *                 Note: this is not the name of the operator, the robot
         *                 can only be connected to one operator, but rather the
         *                 id that was assigned to this webrtc connection.
         * @param msg - The string to send.
         *              Note: This should not be a json or anything, just the
         *              unpackaged message that came from the webrtc system on
         *              the robot
         */
        const sendToOperator = (id: string, msg: string) => {
            if (thisClient.connected === undefined) {
                throw new Error('the robot does not exist');
            }
            const operatorSock = thisClient.connected.rtcSockets.get(id);
            if (operatorSock === undefined) {
                throw new Error('webrtc socket to operator is broken');
            }

            operatorSock.send(msg);
        };
        const pingOperator = (id: string, ws: WebSocket) => {
            if (thisClient.connected === undefined) {
                ws.send(JSON.stringify({ command: 'close', id: id, msg: '' }));
                return;
            }
            const operatorSock = thisClient.connected.rtcSockets.get(id);
            if (operatorSock === undefined) {
                throw new Error('webrtc socket to operator is broken');
            }

            operatorSock.ping();
        };
        const pongOperator = (id: string) => {
            if (thisClient.connected === undefined) {
                throw new Error(
                    'message from robot for a non-existant webrtc connection path',
                );
            }
            const operatorSock = thisClient.connected.rtcSockets.get(id);
            if (operatorSock === undefined) {
                throw new Error('webrtc socket to operator is broken');
            }

            operatorSock.pong();
        };

        // THe socket with the robot was closed, we should kill the webrtc channel
        ws.on('close', () => {
            const sock = thisClient.rtcSockets.get('robot');
            console.log('webrtc connection with robot closing');
            if (sock !== undefined) {
                //sock.close();
                thisClient.rtcSockets.delete('robot');
            }
        });

        // Received a message from the router on the robot
        ws.on('message', (msg: string) => {
            const msgObj = JSON.parse(msg);
            if (msgObj.command === 'msg') {
                sendToOperator(msgObj.id, msgObj.msg);
            } else if (msgObj.command === 'close') {
                console.error(
                    'not yet implemented: robot orders a webrtc socket closed',
                );
            } else if (msgObj.command === 'ping') {
                pingOperator(msgObj.id, ws);
            } else if (msgObj.command === 'pong') {
                pongOperator(msgObj.id);
            }
        });
    }

    /**
     * Handle a new data connection from the robot
     *
     * @remarks
     * This will be coming directly from the rosweb server running on the robot
     */
    async onDataConnection(ws: WebSocket, name: string) {
        const robot = this.clients.get(name);
        if (robot !== undefined) {
            robot.close();
        }
        this.clients.set(name, new Connection(ws, name));
        await db.query('update robots set connected=true where robot_name=$1', [
            name,
        ]);

        console.log('robot ' + name + ' connected data channel');

        ws.on('message', (msg) => {
            const thisRobot = this.clients.get(name);
            if (thisRobot === undefined) {
                throw new Error('disconnected sockets are talking :o');
            }
            thisRobot.onMessage(msg);
        });

        ws.on('close', () => {
            console.log('the robot data connection closed');
            const thisRobot = this.clients.get(name);
            if (thisRobot === undefined) {
                throw new Error('disconnected sockets are talking :o');
                return;
            }
            thisRobot.close();
        });

        ws.on('ping', () => {
            const thisRobot = this.clients.get(name);
            if (thisRobot !== undefined) {
                thisRobot.ping();
            }
        });
        ws.on('pong', () => {
            const thisRobot = this.clients.get(name);
            if (thisRobot !== undefined) {
                thisRobot.pong();
            }
        });
    }
}

/**
 * Manage connections with operators (web interface)
 */
class OperatorConnections extends Connections {
    robots: RobotConnections;
    constructor(robots: RobotConnections) {
        super();
        this.robots = robots;
    }

    onConnection(
        ws: WebSocket,
        request: http.IncomingMessage,
        name: string,
        target: string,
        webrtc: boolean,
    ) {
        console.log(
            'new operator connection from: ' + name + ' webrtc: ' + webrtc,
        );
        if (webrtc) {
            this.onWebRtcConnection(ws, name, target);
        } else {
            this.onDataConnection(ws, name, target);
        }
    }

    // This is when the operator connects its webrtc channel
    onWebRtcConnection(ws: WebSocket, name: string, target: string) {
        const thisClient = this.clients.get(name);
        if (thisClient === undefined) {
            console.error('tried to connect to webrtc before data');
            ws.close();
            return;
        }

        const robot = thisClient.connected;
        if (robot === undefined) {
            console.error(
                'webrtc connection from an operator who is not connected to a robot',
            );
            ws.close();
            return;
        }

        // Each webrtc connection gets a unique identifier to allow the internal routing to work
        const id = uuidv4();

        thisClient.rtcSockets.set(id, ws);

        console.log('Operator ' + name + ' conected webrtc socket');

        // Tell the router on the robot to setup a connection for this
        const robotCon = robot.rtcSockets.get('robot');
        if (robotCon === undefined) {
            console.error(
                'the webrtc connection to the robot router is not yet connected',
            );
            ws.close();
            return;
        }
        // We need to tell the router to open a connection for this channel to the local
        // webrtc server
        robotCon.send(JSON.stringify({ command: 'open', id: id }));
        // TODO: ideally we would want to wait until that ws is open before we finish
        //       establishing the connection with the operator.

        // for sending back to the robot
        const sendToRobot = (id: string, command: string, msg: string) => {
            if (thisClient.connected === undefined) {
                throw new Error(
                    'message from operator for a non-existant webrtc connection path',
                );
            }
            const robotSock = thisClient.connected.rtcSockets.get('robot');
            if (robotSock === undefined) {
                throw new Error('webrtc socket to operataor is broken');
                return;
            }

            const toSend = JSON.stringify({
                id: id,
                command: command,
                msg: msg,
            });

            console.log('sending message to robot: ' + toSend);
            robotSock.send(toSend);
        };

        // THe socket with the operator was closed, we need to tell the router to disconnect on its side, close the socket between here and the router, and remove the socket from the list
        ws.on('close', () => {
            console.log('webrtc socket with the operator closed');
            sendToRobot(id, 'close', '');
            thisClient.rtcSockets.delete(id);
        });

        ws.on('message', (msg: string) => {
            console.log('message from webrtc client: ' + msg);
            sendToRobot(id, 'msg', msg);
        });

        ws.on('ping', () => {
            sendToRobot(id, 'ping', '');
        });
        ws.on('pong', () => {
            sendToRobot(id, 'pong', '');
        });
    }

    async onDataConnection(ws: WebSocket, name: string, target: string) {
        const operator = this.clients.get(name);
        if (operator !== undefined) {
            console.error(
                'received a repeat connection from an operator. Only one connection per operator',
            );
            ws.close();
        }

        const robot = this.robots.clients.get(target);
        if (robot === undefined) {
            console.error('the requested robot is not connected');
            ws.close();
            return;
        }

        if (robot.connected !== undefined) {
            console.error(
                'the robot is already connected. Ony one connection to each robot',
            );
            ws.close();
            return;
        }

        const thisConnection = new Connection(ws, name);
        this.clients.set(name, thisConnection);
        console.log('operator ' + name + ' connected data channel');
        robot.connected = thisConnection;
        thisConnection.connected = robot;
        await db.query('update robots set active_user_id=$1 where name=$2', [
            name,
            target,
        ]);

        ws.on('message', (msg) => {
            const thisOperator = this.clients.get(name);
            if (thisOperator === undefined) {
                throw new Error('disconnected sockets are talking :o');
            }
            thisOperator.onMessage(msg);
        });

        ws.on('close', () => {
            console.log('data connection with the operator closed');
            const thisOperator = this.clients.get(name);
            if (thisOperator === undefined) {
                throw new Error('disconnected sockets are talking :o');
            }
            thisOperator.close();
            this.clients.delete(name);
        });

        ws.on('ping', () => {
            const thisOperator = this.clients.get(name);
            if (thisOperator !== undefined) {
                thisOperator.ping();
            }
        });
        ws.on('pong', () => {
            const thisOperator = this.clients.get(name);
            if (thisOperator !== undefined) {
                thisOperator.pong();
            }
        });
    }
}

class Server {
    server: http.Server;
    robots: RobotConnections;
    operators: OperatorConnections;

    constructor(port: number) {
        this.server = http.createServer(); // The server that will host our sockets
        this.robots = new RobotConnections();
        this.operators = new OperatorConnections(this.robots);
        this.server.on('listening', () => this.listening());
        this.server.on('upgrade', (request, socket, head) =>
            this.upgrade(request, socket, head),
        );
        this.server.listen(port, '0.0.0.0');
    }

    listening() {
        const address = this.server.address();
        if (address === null) {
            throw new Error('something very wrong with starting server');
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
    }

    async upgrade(
        request: http.IncomingMessage,
        socket: net.Socket,
        head: Buffer,
    ) {
        console.log('upgrade request');
        const urlReturn = parseUrl(request.url);
        // TODO: handle logins here
        if (urlReturn === undefined) {
            socket.destroy();
        } else if (urlReturn.originType == 'robot') {
            // For the robots to connect to
            const name = request.headers['robotname'];
            const password = request.headers['robotpassword'];
            let validPassword = false;
            try {
                const {
                    rows,
                } = await db.query(
                    'select password_hash from robots where robot_name=$1',
                    [name],
                );
                validPassword = await bcrypt.compare(
                    password,
                    rows[0]['password_hash'],
                );
            } catch {
                socket.destroy();
                return;
            }

            if (!validPassword) {
                socket.destroy();
                return;
            }
            this.robots.server.handleUpgrade(request, socket, head, (ws) => {
                this.robots.onConnection(
                    ws,
                    request,
                    name as string,
                    urlReturn.target,
                    urlReturn.webrtc,
                );
            });
        } else if (urlReturn.originType === 'operator') {
            // For the clients to hook up to a robot
            const name = 'operator1';
            this.operators.server.handleUpgrade(request, socket, head, (ws) => {
                this.operators.onConnection(
                    ws,
                    request,
                    name,
                    urlReturn.target,
                    urlReturn.webrtc,
                );
            });
        } else {
            console.log('invalid websocket enpoint: ' + urlReturn.originType);
            socket.destroy();
        }
    }
}

const socketPort = 8080; // the port that the socker server is exposed on
const server = new Server(socketPort);
