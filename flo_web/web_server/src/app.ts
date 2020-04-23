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
import ioredis from 'ioredis';
import util from 'util';
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

const sessionParser = session({
    secret: sessionSecret,
    store: sessionStore,
    resave: false,
    saveUninitialized: false,
    cookie: {
        secure: true,
        maxAge: 1000 * 60 * 60 * 48, // 48 hours
    },
});

app.use(sessionParser);

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
    let target = '';
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

/**
 * # REDIS Client Store Structure:
 *
 * - 'robot:name' hash
 *      - 'connected-operator' The id of the connected client
 *      - 'rtc-connected' T/F
 *      - 'data-connected' T/F
 * - 'operator:id' hash
 *      - 'connected-robot' The name of the connected robot
 *      - 'data-connected' T/F
 * - 'operator:id:webrtcIDs' List of the IDs for webrtc
 *
 * ## Pub/Sub
 * - 'robot:<name>:outgoing-data' the standard data going out from the robot
 * - 'robot:<name>:incoming-data' the standard data going to the robot
 * - 'robot:<name>:outgoing-commands' The commands going to the operator
 * - 'robot:<name>:incoming-commands' the commands to the robot (for socket management)
 * - 'robot:<name>:outgoing-commands-rtc:<id>' the webrtc commands going out from the robot
 * - 'robot:<name>:outgoing-data-rtc:<id>' the webrtc data going out from the robot
 * - 'robot:<name>:incoming-data-rtc' The data for the robot from the operator
 *
 */
const ClientStore = () => {
    return new ioredis({
        host: 'client-store',
        port: 6379,
    });
};

class Server {
    server: http.Server;
    sockets: Map<string, WebSocket>;
    wsServer: WebSocket.Server;

    constructor(port: number) {
        this.server = http.createServer(); // The server that will host our sockets
        this.server.on('listening', () => this.listening());
        this.server.on('upgrade', (request, socket, head) =>
            this.upgrade(request, socket, head),
        );
        this.wsServer = new WebSocket.Server({ noServer: true });
        this.server.listen(port, '0.0.0.0');
        this.sockets = new Map();
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
        const urlReturn = parseUrl(request.url);
        const handleUpgradePromise = (
            request: http.IncomingMessage,
            socket: net.Socket,
            head: Buffer,
        ): Promise<WebSocket> => {
            return new Promise((resolve, reject) => {
                this.wsServer.handleUpgrade(request, socket, head, (ws) => {
                    resolve(ws);
                });
            });
        };

        if (urlReturn === undefined) {
            socket.destroy();
            return;
        }
        const rdb = ClientStore();
        const rpub = ClientStore();
        const rsub = ClientStore();
        let cmdC: string;
        let msgC: string;

        console.log(
            `upgrade request for ${urlReturn.originType} with webrtc? ${urlReturn.webrtc}`,
        );

        if (urlReturn.originType == 'robot') {
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

            const ws = await handleUpgradePromise(request, socket, head);

            const webrtcname = `robot:${name}:webrtc`;
            const dataname = `robot:${name}:data`;

            // check if robot is available:
            const channels = await rpub.pubsub(
                'channels',
                `robot:${name}:outgoing-commands*`,
            );
            console.log('connected channels');
            console.log(channels);
            if (channels.length === 0) {
                db.query(
                    'update robots set active_user_id=$1 where robot_name=$2',
                    [null, name],
                );
                console.log('no operators connected');
            }

            if (urlReturn.webrtc) {
                this.sockets.set(webrtcname, ws);
                rdb.hset(`robot:${name}`, 'rtc-connected', 'true');

                ws.on('close', () => {
                    rsub.unsubscribe();
                    this.sockets.delete(webrtcname);
                    db.query(
                        'update robots set connected=$1 where robot_name=$2',
                        [false, name],
                    );
                    rdb.hset(`robot:${name}`, 'webrtc-connected', 'false');
                });

                ws.on('message', (msg: string) => {
                    const msgData = JSON.parse(msg);
                    const command = msgData['command'];
                    const channelID = msgData['id'];
                    cmdC = `robot:${name}:outgoing-commands-rtc:${channelID}`;
                    msgC = `robot:${name}:outgoing-data-rtc:${channelID}`;
                    if (command === 'close') {
                        rpub.publish(cmdC, 'close');
                    } else if (command === 'ping') {
                        rpub.publish(cmdC, 'ping');
                    } else if (command === 'message') {
                        rpub.publish(msgC, msgData['msg']);
                    }
                });

                const wrtcC = `robot:${name}:incoming-data-rtc`;
                rsub.subscribe(wrtcC);
                rsub.on('message', (channel, message) => {
                    if (channel === wrtcC) {
                        console.log('sending robot rtc msg: ' + message);
                        ws.send(message);
                    }
                });
            } else {
                cmdC = `robot:${name}:outgoing-commands`;
                msgC = `robot:${name}:outgoing-data`;

                this.sockets.set(dataname, ws);
                rdb.hset(`robot:${name}`, 'data-connected', 'true');

                ws.on('close', async () => {
                    rpub.publish(cmdC, 'close');
                    rsub.unsubscribe();
                    this.sockets.delete(dataname);
                    db.query(
                        'update robots set connected=$1 where robot_name=$2',
                        [false, name],
                    );
                    rdb.hset(`robot:${name}`, 'data-connected', 'false');
                });
                ws.on('message', (msg: string) => {
                    rpub.publish(msgC, msg);
                });
                ws.on('ping', () => {
                    rpub.publish(cmdC, 'ping');
                });

                const dataC = `robot:${name}:incoming-data`;
                const commandC = `robot:${name}:incoming-commands`;
                rsub.subscribe(dataC);
                rsub.subscribe(commandC);
                rsub.on('message', (channel, message) => {
                    console.log('sending robot msg: ' + message);
                    if (channel === dataC) {
                        ws.send(message);
                    } else if (channel === commandC) {
                        if (message === 'close') {
                            ws.close();
                        } else if (message === 'ping') {
                            ws.ping();
                        }
                    }
                });
            }

            const dataConnected = await rdb.hget(
                `robot:${name}`,
                'data-connected',
            );
            const rtcConnected = await rdb.hget(
                `robot:${name}`,
                'rtc-connected',
            );

            console.log(dataConnected);
            console.log(rtcConnected);

            if (dataConnected && rtcConnected) {
                db.query('update robots set connected=$1 where robot_name=$2', [
                    true,
                    name,
                ]);
            }
        } else if (urlReturn.originType === 'operator') {
            // For the clients to hook up to a robot
            sessionParser(
                request as express.Request,
                {} as express.Response,
                async () => {
                    const id = (request as express.Request).session!.userID;
                    const targetRobot = urlReturn.target;
                    if (!id) {
                        socket.destroy();
                        return;
                    }

                    const {
                        rows,
                    } = await db.query(
                        'select count(*) from robot_permissions rp ' +
                            'left join robots r on r.id = rp.robot_id ' +
                            'where rp.user_id =$1 and r.robot_name =$2',
                        [id, targetRobot],
                    );

                    if (rows[0] < 1) {
                        console.log('user not authorized');
                        socket.destroy();
                        return;
                    }

                    console.log('user is authorized');

                    // make sure the robot is available and get if it is
                    const res = await rdb
                        .multi()
                        .hget(`robot:${targetRobot}`, 'data-connected') //0
                        .hget(`robot:${targetRobot}`, 'rtc-connected') //1
                        .hget(`robot:${targetRobot}`, 'connected-operator') //2
                        .hsetnx(
                            `robot:${targetRobot}`,
                            'connected-operator',
                            id,
                        )
                        .exec();

                    // If it isn't avaialable, ok if we are already connected
                    if (res[3][1] === 0) {
                        if (parseInt(res[2][1]) !== id) {
                            console.log(
                                `${id}: another user is connected (${res[2][1]}), disconnectiong`,
                            );
                            socket.destroy;
                            return;
                        }
                    }

                    // reserve the robot on postgres
                    db.query(
                        'update robots set active_user_id=$1 where robot_name=$2',
                        [id, targetRobot],
                    );
                    rdb.hset(`operator:${id}`, 'connected-robot', targetRobot);

                    console.log('connected to robot');

                    const ws = await handleUpgradePromise(
                        request,
                        socket,
                        head,
                    );

                    if (urlReturn.webrtc) {
                        const channelID = uuidv4();
                        const key = `operator:${id}:webrtc:${channelID}`;
                        this.sockets.set(key, ws);

                        rdb.sadd(`operator:${id}:webrtcIDs`, channelID);

                        rpub.publish(
                            `robot:${targetRobot}:incoming-data-rtc`,
                            JSON.stringify({ command: 'open', id: channelID }),
                        );

                        ws.on('close', async () => {
                            rpub.publish(
                                `robot:${targetRobot}:incoming-data-rtc`,
                                JSON.stringify({
                                    command: 'close',
                                    id: channelID,
                                }),
                            );
                            this.sockets.delete(key);
                            rsub.unsubscribe();
                            rdb.srem(`operator:${id}:webrtcIDs`, channelID);
                        });

                        ws.on('ping', () => {
                            rpub.publish(
                                `robot:${targetRobot}:incoming-data-rtc`,
                                JSON.stringify({
                                    command: 'ping',
                                    id: channelID,
                                }),
                            );
                        });

                        ws.on('message', (message) => {
                            console.log(
                                'webrtc message from operator: ' + message,
                            );
                            rpub.publish(
                                `robot:${targetRobot}:incoming-data-rtc`,
                                JSON.stringify({
                                    command: 'msg',
                                    id: channelID,
                                    msg: message,
                                }),
                            );
                        });

                        cmdC = `robot:${targetRobot}:outgoing-commands-rtc:${channelID}`;
                        msgC = `robot:${targetRobot}:outgoing-data-rtc:${channelID}`;
                    } else {
                        const key = `operator:${id}:data`;
                        this.sockets.set(key, ws);
                        // onClose:
                        // - remove from this list
                        // - tell the robot to close
                        ws.on('close', async () => {
                            rpub.publish(
                                `robot:${targetRobot}:incoming-commands`,
                                'close',
                            );
                            this.sockets.delete(key);
                            rsub.unsubscribe();
                        });
                        // onMessage
                        // - put it in the robot incoming queue
                        ws.on('message', async (msg: string) => {
                            console.log('operator sent message: ' + msg);
                            rpub.publish(
                                `robot:${targetRobot}:incoming-data`,
                                msg,
                            );
                        });
                        // onPing
                        // - put it in the robot incoming command queue
                        ws.on('ping', async () => {
                            rpub.publish(
                                `robot:${targetRobot}:incoming-commands`,
                                'ping',
                            );
                        });
                        // while open:
                        // - listen to robot outgoing commands
                        //      - close: close this
                        //      - ping: pass on ping
                        cmdC = `robot:${targetRobot}:outgoing-commands`;
                        msgC = `robot:${targetRobot}:outgoing-data`;
                    }
                    rsub.subscribe(cmdC);
                    rsub.subscribe(msgC);
                    rsub.on('message', (channel, message) => {
                        if (channel === cmdC) {
                            if (message === 'close') {
                                ws.close();
                            } else if (message === 'ping') {
                                ws.ping();
                            }
                        } else if (channel === msgC) {
                            ws.send(message);
                        }
                    });
                },
            );
        } else {
            console.log('invalid websocket enpoint: ' + urlReturn.originType);
            socket.destroy();
        }
    }
}

const socketPort = 8080; // the port that the socker server is exposed on
const server = new Server(socketPort);
