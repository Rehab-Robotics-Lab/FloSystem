// lib/app.ts
import express from 'express';
import http from 'http';
import WebSocket from 'ws';
import url from 'url';
import { v4 as uuidv4 } from 'uuid';
//import bcrypt from "bcrypt";
//import passport from "passport";
//import session from "express-session";

// TODO: for testing only, very dangerous
process.env['NODE_TLS_REJECT_UNAUTHORIZED'] = '0';

const socketPort = 9091;
const webUri = '192.168.1.7'; // process.env.FLO_SERVER_IP; //TODO: bring in as an environment var
const rtcServer = 'localhost';

function WebClient(url: string) {
    const connections: Record<string, WebSocket> = {};
    let client: WebSocket | undefined;
    let connection = false;
    let connecting = false;
    let timeout: ReturnType<typeof setTimeout> | undefined;
    let timeoutPing: ReturnType<typeof setInterval> | undefined;
    let sendUp: (
        target: string,
        command: string,
        msg: string,
    ) => void | undefined;

    const init = () => {
        console.log('connecting to server');
        connecting = false;
        if (client !== undefined) {
            client.removeAllListeners();
        }
        client = new WebSocket(url);
        if (timeoutPing !== undefined) {
            clearInterval(timeoutPing);
        }
        timeoutPing = setInterval(() => {
            if (client !== undefined) {
                client.ping();
            }
        }, 1000);

        const heartbeat = () => {
            console.log('hearbeat');
            if (timeout !== undefined) {
                clearTimeout(timeout);
                timeout = undefined;
            }
            timeout = setTimeout(() => (client as WebSocket).terminate(), 5000);
        };

        sendUp = (target: string, command: string, msg: string) => {
            if (client !== undefined) {
                const toSend = JSON.stringify({
                    target: target,
                    command: command,
                    msg: msg,
                });
                console.log('sending server: ' + toSend);
                client.send(toSend);
            } else {
                console.error(
                    'tried to send a message up to the server, but not connected',
                );
            }
        };

        client.on('ping', () => {
            console.log('received ping from server');
            heartbeat();
        });

        client.on('pong', () => {
            console.log('recieved pong from server');
            heartbeat();
        });

        client.on('open', () => {
            console.log('opened connection');
            heartbeat();
        });

        client.on('message', (msg: string) => {
            console.log('got message from server: ' + msg);
            heartbeat();

            const msgObj = JSON.parse(msg);
            const command = msgObj['command'];

            if (command === 'open') {
                const ws = new WebSocket(
                    'ws://' + rtcServer + ':' + socketPort,
                );
                const target = msgObj['target'];
                connections[target] = ws;
                ws.on('message', (msg: string) => {
                    sendUp(target, 'msg', msg);
                });
                ws.on('close', () => {
                    sendUp(target, 'close', '');
                });
            } else if (command === 'msg') {
                connections[msgObj['target']].send(msgObj['msg']);
            } else if (command === 'close') {
                connections[msgObj['target']].close();
                delete connections[msgObj['target']];
            } else {
                console.error('got an invalid command');
            }
        });

        client.on('error', () => {
            console.log('error reported');
            if (connecting === false) {
                connecting = true;
                setTimeout(() => {
                    init();
                }, 1000);
            }
        });
        client.on('close', () => {
            console.log('close reported');
            if (connecting === false) {
                connecting = true;
                setTimeout(() => {
                    init();
                }, 1000);
            }
        });
    };
    init();
}

let webClient = WebClient('wss://' + webUri + '/host/webrtc');
