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
const webUri = '192.168.1.7'; //TODO: bring in as an environment var

const server = http.createServer();

const robotClient = new WebSocket('ws://localhost:' + socketPort);
const webClient = new WebSocket('ws://' + webUri + '/host/webrtc');
