// lib/app.ts
import express from 'express';
import http from 'http';
import WebSocket from 'ws';
import url from 'url';
import { v4 as uuidv4 } from 'uuid';
//import bcrypt from "bcrypt";
//import passport from "passport";
//import session from "express-session";
import winston from 'winston';

const logger = winston.createLogger({
    level: 'info',
    format: winston.format.prettyPrint(),
    transports: [
        new winston.transports.Console({
        }),
    ],
});

class ReconnectigWS {
    /**
     * Connect to and manage the connection with a remote websocket server.
     * Reconnect automatically. And ping the server frequently to make sure
     * that the connection is still alive.
     *
     * @remarks
     * Will consider any communication from the server (message, connection,
     * pong, ping) as valid and use those to reset the connection timeout
     * timer.
     *
     * The client pings the server on a regular frequency (defined in
     * pingFreq) begining after the socket connects.
     *
     * @param url - The url of the remote socket server
     * @param onMessage - What you want to do when a new message arrives
     * @param onClose - What you want to do when the socket has been closed
     *                  Note: the socket will reconnect regardless of what you
     *                  put here
     * @param onError - What you want to do when the socket has had an error
     *                  Note: the socket will reconnect regardless of what you
     *                  put here
     * @param pingFreq - The frequency with which you want the server to be pinged (ms)
     * @param reconnectDelay - The delay betwen reconect attempts (ms)
     * @param connectionTimeout - The amount of time to wait to hear from the
     *                            server before concluding that the connection
     *                            is dead. (should be larger than the ping
     *                            freq)
     */
    sock: WebSocket | undefined;
    timeout: ReturnType<typeof setTimeout> | undefined;
    pingTimer: ReturnType<typeof setInterval> | undefined;
    connection: boolean;
    connecting: boolean;
    url: string;
    onMessage: (msg: string) => void;
    onClose: () => void;
    onError: (err: Error) => void;
    pingFreq: number;
    reconnectDelay: number;
    connectionTimeout: number;
    buffer: string[];

    constructor(
        url: string,
        onMessage: (msg: string) => void = (msg) => {},
        onClose: () => void = () => {},
        onError: (err: Error) => void = () => {},
        pingFreq: number = 1000,
        reconnectDelay: number = 1000,
        connectionTimeout: number = 5000,
    ) {
        this.url = url;
        this.connection = false;
        this.connecting = false;
        this.onMessage = onMessage;
        this.onClose = onClose;
        this.onError = onError;
        this.pingFreq = pingFreq;
        this.reconnectDelay = reconnectDelay;
        this.connectionTimeout = connectionTimeout;
        this.buffer = [];

        this.connect();
    }

    connect() {
        logger.info('connecting to server');
        this.connecting = false;
        if (this.sock !== undefined) {
            this.sock.removeAllListeners();
        }
        const robotName = process.env['ROBOT_NAME'];
        const robotPassword = process.env['ROBOT_PASSWORD'];
        logger.info(`connecting to ${this.url} with name: ${robotName}`);
        this.sock = new WebSocket(this.url, {
            headers: {
                robotName: robotName,
                robotPassword: robotPassword,
            },
        });

        if (this.pingTimer !== undefined) {
            clearInterval(this.pingTimer);
        }

        this.sock.on('message', (msg: string) => {
            logger.info('message from server', {msg});
            this.heartbeat();
            this.onMessage(msg);
        });

        this.sock.on('open', () => {
            logger.info('socket opened with server');
            this.heartbeat();
            // Setup ping to go out every so often.
            this.pingTimer = setInterval(() => {
                if (this.sock !== undefined) {
                    this.sock.ping();
                }
            }, this.pingFreq);
            for (const msg of this.buffer) {
                logger.info('sending buffered message to server',{msg});
                (this.sock as WebSocket).send(msg);
            }
        });

        this.sock.on('error', (err) => {
            logger.error('error with server ws', {err});
            this.onError(err);
            this.reconnect();
        });

        this.sock.on('close', () => {
            logger.info('server ws closing');
            this.onClose();
            this.reconnect();
        });

        this.sock.on('ping', () => {
            logger.silly('ping from server');
            this.heartbeat();
        });

        this.sock.on('pong', () => {
            logger.silly('pong from server');
            this.heartbeat();
        });
    }

    reconnect() {
        if (this.connecting === false) {
            this.connecting = true;
            setTimeout(() => {
                this.connect();
            }, this.reconnectDelay);
        }
    }

    heartbeat() {
        //console.log('hearbeat');
        if (this.timeout !== undefined) {
            clearTimeout(this.timeout);
            this.timeout = undefined;
        }
        this.timeout = setTimeout(() => {
            if (this.sock !== undefined) {
                this.sock.terminate();
            }
        }, this.connectionTimeout);
    }

    send(msg: string | object) {
        if (typeof msg === 'object') {
            msg = JSON.stringify(msg);
        }
        if (this.sock == undefined) {
            logger.error('Socket is not connected to server');
            return;
        }
        logger.info('sending message to server', {msg});
        if (this.sock.readyState === WebSocket.OPEN) {
            this.sock.send(msg);
        } else {
            this.buffer.push(msg);
        }
    }
}

const socketPort = 9091;
const webUri =
    'wss://' + (process.env.FLO_SERVER_IP || 'lilflo.com') + '/host/webrtc';
const rtcServer = 'localhost';
const connections: Record<string, WebSocket> = {};
const bufferedMsgs: Map<string, [string]> = new Map();

const connection = new ReconnectigWS(webUri);

function sendUp(id: string, command: string, msg: string) {
    const pack = {
        id: id,
        command: command,
        msg: msg,
    };
    logger.info('sending to server',{msg})
    connection.send(pack);
}

connection.onMessage = (msg: string): void => {
    logger.info('message received from server', {msg});
    const msgObj = JSON.parse(msg);
    const id = msgObj['id'];
    const command = msgObj['command'];

    if (command === 'open') {
        const ws = new WebSocket(
            'ws://' + rtcServer + ':' + socketPort + '/webrtc',
        );
        logger.info('opened new local websocket', {id});
        connections[id] = ws;
        ws.on('message', (msg: string) => {
            sendUp(id, 'msg', msg);
        });
        ws.on('close', () => {
            sendUp(id, 'close', '');
        });
        ws.on('open', () => {
            const thisBuffer = bufferedMsgs.get(id);
            bufferedMsgs.delete(id);
            logger.info('ws to roswebrtc opened, clearing bufer', {thisBuffer});
            if (thisBuffer !== undefined) {
                for (msg of thisBuffer) {
                    logger.info('sending from buffer', {msg});
                    ws.send(msg);
                }
            }
        });
        ws.on('ping', () => {
            sendUp(id, 'ping', '');
        });
        ws.on('pong', () => {
            sendUp(id, 'ping', '');
        });
    } else if (command === 'msg') {
        const toSend = msgObj['msg'];
        logger.info('sending message to roswebrtc', {id:msgObj['id'],msg:toSend});
        if (connections[id].readyState === WebSocket.CONNECTING) {
            const existingBuffer = bufferedMsgs.get(id);
            if (existingBuffer === undefined) {
                bufferedMsgs.set(id, [toSend]);
            } else {
                existingBuffer.push(toSend);
            }
        } else {
            connections[msgObj['id']].send(toSend);
        }
    } else if (command === 'close') {
        const thisConnection = connections[id];
        if (thisConnection !== undefined) {
            logger.info('closing ws connection to webrtc ros id', {id});
            thisConnection.close();
            delete connections[id];
        } else {
            logger.info(
                'ws connectin commanded to close by server, but already closed. ID',
                {id},
            );
        }
    } else if (command === 'ping') {
        connections[id].ping();
    } else if (command === 'pong') {
        connections[id].pong();
    } else {
        logger.info('got an invalid command');
    }
};
