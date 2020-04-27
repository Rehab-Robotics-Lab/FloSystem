import winston from 'winston';
import session from 'express-session';
import redis from 'redis';
import connectRedis from 'connect-redis';
import url from 'url';
import ioredis from 'ioredis';
import http from 'http';
import net from 'net';
import WebSocket from 'ws';

const logger = winston.createLogger({
    level: 'info',
    format: winston.format.json(),
    transports: [
        //
        // - Write to all logs with level `info` and below to `combined.log`
        // - Write all logs error (and below) to `error.log`.
        //
        //new winston.transports.File({
        //filename: '~/logs/error.log',
        //level: 'error',
        //}),
        //new winston.transports.File({ filename: '~/logs/combined.log' }),
        new winston.transports.Console({
            format: winston.format.simple(),
            level: 'verbose',
        }),
    ],
});

export { logger };

const RedisStore = connectRedis(session);
const sessionStore = new RedisStore({
    client: redis.createClient({ host: 'session-store', port: 6379 }),
});

const sessionSecret = process.env.SESSION_SECRET || 'secret';
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

export { sessionParser };

interface UrlReturn {
    originType: 'robot' | 'operator';
    target: string;
    webrtc: boolean;
}
interface ParseUrl {
    (reqUrl: string | undefined): undefined | UrlReturn;
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

const parseUrl: ParseUrl = function (reqUrl) {
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
};
export { parseUrl };

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
const ClientStore = (): ioredis.Redis => {
    return new ioredis({
        host: 'client-store',
        port: 6379,
    });
};
export { ClientStore };

interface Upgrade {
    (request: http.IncomingMessage, socket: net.Socket, head: Buffer): void;
}

interface HandleUpgradePromise {
    (request: http.IncomingMessage, socket: net.Socket, head: Buffer): Promise<
        WebSocket
    >;
}

interface ParseIncoming {
    (
        urlReturn: UrlReturn,
        request: http.IncomingMessage,
        socket: net.Socket,
        head: Buffer,
        handleUpgradePromise: HandleUpgradePromise,
        addSocket: (name: string, ws: WebSocket) => void,
        removeSocket: (name: string) => void,
    ): void;
}

class Server {
    server: http.Server;
    sockets: Map<string, WebSocket>;
    wsServer: WebSocket.Server;
    parseIncoming: ParseIncoming;

    constructor(port: number, parseIncoming: ParseIncoming) {
        this.server = http.createServer(); // The server that will host our sockets
        this.server.on('listening', () => this.listening());
        this.server.on('upgrade', (request, socket, head) =>
            this.upgrade(request, socket, head),
        );
        this.sockets = new Map();
        this.parseIncoming = parseIncoming;
        this.wsServer = new WebSocket.Server({ noServer: true });
        this.server.listen(port, '0.0.0.0');
    }

    listening(): void {
        const address = this.server.address();
        if (address === null) {
            logger.error('Started server with no address');
            throw new Error('something very wrong with starting server');
        }
        if (typeof address === 'string') {
            logger.info('Socket server listening at: ' + address);
        } else {
            logger.info(
                'Socket server listening at: ' +
                    address.address +
                    ':' +
                    address.port,
            );
        }
    }

    addSocket(name: string, ws: WebSocket) {
        if (this.sockets.has(name)) {
            throw Error('socked already in list');
        }
        this.sockets.set(name, ws);
    }

    removeSocket(name: string) {
        this.sockets.delete(name);
    }

    async upgrade(
        request: http.IncomingMessage,
        socket: net.Socket,
        head: Buffer,
    ): Promise<void> {
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

        logger.debug(
            `upgrade request for ${urlReturn.originType} with webrtc? ${urlReturn.webrtc}`,
            urlReturn,
        );

        this.parseIncoming(
            urlReturn,
            request,
            socket,
            head,
            (
                request: http.IncomingMessage,
                socket: net.Socket,
                head: Buffer,
            ) => {
                return handleUpgradePromise(request, socket, head);
            },
            (name: string, ws: WebSocket) => {
                this.addSocket(name, ws);
            },
            (name: string) => {
                this.removeSocket(name);
            },
        );
    }
}

export { Server, HandleUpgradePromise, ParseIncoming };

/**
 * Helper function to kill a websocket if it isn't being talked to
 *
 * @remarks
 * Will start the first time that hearbeat is called.
 * Heartbeat should be called each time the socket is heard from
 * When the websocket closes, the cancel function should be called
 */
function killOnDisconnect(
    ws: WebSocket,
    pingFreq: number = 1000,
    connectionTimeout: number = 5000,
) {
    const localLogger = logger.child({
        source: 'killOnDisconnect',
    });

    let pingTimer: undefined | ReturnType<typeof setInterval> = undefined;
    let timeout: undefined | ReturnType<typeof setTimeout> = undefined;

    const cancel = () => {
        localLogger.verbose('cancel');
        if (pingTimer !== undefined) {
            clearTimeout(pingTimer);
        }
        if (timeout !== undefined) {
            clearTimeout(timeout);
        }
    };

    const heartbeat = () => {
        localLogger.silly('heartbeat');
        if (timeout !== undefined) {
            clearTimeout(timeout);
            timeout = undefined;
        }
        timeout = setTimeout(() => {
            localLogger.info('killing socket');
            ws.terminate();
            cancel();
        }, connectionTimeout);
        if (pingTimer !== undefined) {
            clearInterval(pingTimer);
            pingTimer = undefined;
        }
        pingTimer = setInterval(() => {
            ws.ping();
        }, pingFreq);
    };

    return {
        heartbeat: heartbeat,
        cancel: cancel,
    };
}
export { killOnDisconnect };
