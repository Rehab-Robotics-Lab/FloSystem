import express from 'express';
import http from 'http';
import WebSocket from 'ws';
import { v4 as uuidv4 } from 'uuid';
import net from 'net';
import bcrypt from 'bcrypt';
import * as db from './db';
import ioredis from 'ioredis';
import {
    sessionParser,
    logger,
    parseUrl,
    ClientStore,
    ParseIncoming,
    Server,
    killOnDisconnect,
} from './util';

const parseIncoming: ParseIncoming = async function (
    urlReturn,
    request,
    socket,
    head,
    handleUpgradePromise,
    addSocket,
    removeSocket,
) {
    const rdb = ClientStore();
    const rpub = ClientStore();
    const rsub = ClientStore();
    let cmdC: string;
    let msgC: string;

    if (urlReturn.originType !== 'robot') {
        logger.warn('invalid websocket enpoint: ', urlReturn.originType);
        socket.destroy();
        return;
    }
    // For the robots to connect to
    const name = request.headers['robotname'];
    const password = request.headers['robotpassword'];

    const localLogger = logger.child({
        source: 'robot',
        name: name,
        wsType: urlReturn.webrtc ? 'webrtc' : 'data',
    });

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
        localLogger.error('error while authenticating robot');
        socket.destroy();
        return;
    }

    if (!validPassword) {
        localLogger.error('invalid auth from robot');
        socket.destroy();
        return;
    }

    const ws = await handleUpgradePromise(request, socket, head);

    const killer = killOnDisconnect(ws);
    ws.on('pong', killer.heartbeat);
    killer.heartbeat();

    localLogger.debug(`succesfully upgraded connection from ${name}`);

    const webrtcname = `robot:${name}:webrtc`;
    const dataname = `robot:${name}:data`;

    const checkDisconnect = async (): Promise<void> => {
        const [dataConnected, rtcConnected] = await Promise.all([
            rdb.hget(`robot:${name}`, 'data-connected'),
            rdb.hget(`robot:${name}`, 'rtc-connected'),
        ]);
        localLogger.verbose('check disconnecct', {
            dataConnected: dataConnected,
            rtcConnected: rtcConnected,
        });
        if (
            dataConnected === 'false' ||
            dataConnected === null ||
            rtcConnected === 'false' ||
            rtcConnected === null
        ) {
            db.query('update robots set connected=$1 where robot_name=$2', [
                false,
                name,
            ]);
            localLogger.info('robot disconnected');
        }
    };
    if (urlReturn.webrtc) {
        addSocket(webrtcname, ws);
        rdb.hset(`robot:${name}`, 'rtc-connected', 'true');

        ws.on('close', () => {
            localLogger.info('ws close');
            rsub.unsubscribe();
            removeSocket(webrtcname);
            rdb.hset(`robot:${name}`, 'rtc-connected', 'false');
            checkDisconnect();
            killer.cancel();
        });

        ws.on('message', (msg: string) => {
            localLogger.silly('ws message', msg);
            const msgData = JSON.parse(msg);
            const command = msgData['command'];
            const channelID = msgData['id'];
            cmdC = `robot:${name}:outgoing-commands-rtc:${channelID}`;
            msgC = `robot:${name}:outgoing-data-rtc:${channelID}`;
            if (command === 'close') {
                rpub.publish(cmdC, 'close');
            } else if (command === 'ping') {
                rpub.publish(cmdC, 'ping');
            } else if (command === 'msg') {
                rpub.publish(msgC, msgData['msg']);
            }
            killer.heartbeat();
        });

        const wrtcC = `robot:${name}:incoming-data-rtc`;
        rsub.subscribe(wrtcC);
        rsub.on('message', (channel, message) => {
            localLogger.silly('incoming-data-rtc message', {
                channel,
                message,
            });
            if (channel === wrtcC) {
                ws.send(message);
            }
        });
    } else {
        cmdC = `robot:${name}:outgoing-commands`;
        msgC = `robot:${name}:outgoing-data`;

        addSocket(dataname, ws);
        rdb.hset(`robot:${name}`, 'data-connected', 'true');

        ws.on('close', async () => {
            localLogger.debug('ws close');
            rpub.publish(cmdC, 'close');
            rsub.unsubscribe();
            removeSocket(dataname);
            rdb.hset(`robot:${name}`, 'data-connected', 'false');
            checkDisconnect();
            killer.cancel();
        });
        ws.on('message', (msg: string) => {
            localLogger.silly('ws message', msg);
            rpub.publish(msgC, msg);
            killer.heartbeat();
        });
        ws.on('ping', () => {
            localLogger.silly('ws ping');
            rpub.publish(cmdC, 'ping');
            killer.heartbeat();
        });

        const dataC = `robot:${name}:incoming-data`;
        const commandC = `robot:${name}:incoming-commands`;
        rsub.subscribe(dataC);
        rsub.subscribe(commandC);
        rsub.on('message', (channel, message) => {
            localLogger.silly('redis sub message', {
                channel,
                message,
            });
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

    const [dataConnected, rtcConnected] = await Promise.all([
        rdb.hget(`robot:${name}`, 'data-connected'),
        rdb.hget(`robot:${name}`, 'rtc-connected'),
    ]);

    localLogger.debug('data connected', dataConnected);
    localLogger.debug('rtc connected', rtcConnected);

    if (dataConnected && rtcConnected) {
        db.query('update robots set connected=$1 where robot_name=$2', [
            true,
            name,
        ]);
    }
};

const server = new Server(8080, parseIncoming);
