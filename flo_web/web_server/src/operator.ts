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
    if (urlReturn.originType !== 'operator') {
        logger.warn('invalid websocket enpoint: ', urlReturn.originType);
        socket.destroy();
        return;
    }
    // For the clients to hook up to a robot
    sessionParser(
        request as express.Request,
        {} as express.Response,
        async () => {
            const id = (request as express.Request).session!.userID;
            const targetRobot = urlReturn.target;
            const localLogger = logger.child({
                source: 'operator',
                id: id,
                target: targetRobot,
                wsType: urlReturn.webrtc ? 'webrtc' : 'data',
            });
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
                localLogger.info('user not authorized');
                socket.destroy();
                return;
            }

            localLogger.verbose('user is authorized');

            // make sure the robot is available and get if it is
            const res = await rdb
                .multi()
                .hget(`robot:${targetRobot}`, 'data-connected') //0
                .hget(`robot:${targetRobot}`, 'rtc-connected') //1
                .hget(`robot:${targetRobot}`, 'connected-operator') //2
                .hsetnx(`robot:${targetRobot}`, 'connected-operator', id)
                .exec();

            // If it isn't avaialable, ok if we are already connected
            if (res[3][1] === 0) {
                if (parseInt(res[2][1]) !== id) {
                    localLogger.verbose(
                        'another user is connected, disconnectiong',
                        res[2][1],
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

            localLogger.verbose('connected to robot');

            const ws = await handleUpgradePromise(request, socket, head);

            if (urlReturn.webrtc) {
                const channelID = uuidv4();
                const key = `operator:${id}:webrtc:${channelID}`;
                addSocket(key, ws);

                rdb.sadd(`operator:${id}:webrtcIDs`, channelID);

                rpub.publish(
                    `robot:${targetRobot}:incoming-data-rtc`,
                    JSON.stringify({ command: 'open', id: channelID }),
                );

                ws.on('close', async () => {
                    localLogger.debug('ws close');
                    rpub.publish(
                        `robot:${targetRobot}:incoming-data-rtc`,
                        JSON.stringify({
                            command: 'close',
                            id: channelID,
                        }),
                    );
                    removeSocket(key);
                    rsub.unsubscribe();
                    rdb.srem(`operator:${id}:webrtcIDs`, channelID);
                });

                ws.on('ping', () => {
                    localLogger.silly('ws ping');
                    rpub.publish(
                        `robot:${targetRobot}:incoming-data-rtc`,
                        JSON.stringify({
                            command: 'ping',
                            id: channelID,
                        }),
                    );
                });

                ws.on('message', (message) => {
                    localLogger.silly('ws msg', message);
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
                addSocket(key, ws);
                // onClose:
                // - remove from this list
                // - tell the robot to close
                ws.on('close', () => {
                    localLogger.debug('ws close');
                    rpub.publish(
                        `robot:${targetRobot}:incoming-commands`,
                        'close',
                    );
                    removeSocket(key);
                    rsub.unsubscribe();
                });
                // onMessage
                // - put it in the robot incoming queue
                ws.on('message', (msg: string) => {
                    localLogger.silly('ws message', msg);
                    rpub.publish(`robot:${targetRobot}:incoming-data`, msg);
                });
                // onPing
                // - put it in the robot incoming command queue
                ws.on('ping', () => {
                    localLogger.silly('ws ping');

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
                localLogger.silly('redis pub msg', channel, message);
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
};
const server = new Server(8080, parseIncoming);
