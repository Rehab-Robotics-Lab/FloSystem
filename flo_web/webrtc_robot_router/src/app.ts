// lib/app.ts
import express from 'express';
import http from 'http';
import WebSocket from 'ws';
import url from 'url';
import { v4 as uuidv4 } from 'uuid';
//import bcrypt from "bcrypt";
//import passport from "passport";
//import session from "express-session";

const socketPort = 9091;
const webUri = 'lilflo.com/host/webrtc'; //TODO: bring in as an environment var

const connections: Record<string, WebSocket> = {};

let webClient = new WebSocket('wss://' + webUri + '/host/webrtc');

webClient.on('close', () => {
    webClient = new WebSocket('wss://' + webUri + '/host/webrtc');
});

webClient.on('message', (msg: string) => {
    const msgObj = JSON.parse(msg);
    const command = msgObj['command'];

    const sendUp = (target: string, command: string, msg: string) => {
        webClient.send(
            JSON.stringify({ target: target, command: command, msg: msg }),
        );
    };

    if (command === 'open') {
        const ws = new WebSocket('ws://localhost:' + socketPort);
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
    } else {
        console.error('got an invalid command');
    }
});
