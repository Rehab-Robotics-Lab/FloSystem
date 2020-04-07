// lib/app.ts
import express from 'express';
import http from 'http';
import WebSocket from 'ws';
import url from 'url';
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
    connected: string;
    name: string;
}

interface Client {
    socket: WebSocket;
    name: string;
    connected: string;
}

const robotWS: Record<string, Robot> = {};
const clientWS: Record<string, Client> = {};

const closeClient = (client: string): void => {
    if (!(client in clientWS)) {
        console.error('tried to remove non-existant client');
        return;
    }
    const thisClient = clientWS[client];

    if (thisClient.connected in robotWS) {
        // TODO: let the robot know that the client has disconnected
        robotWS[thisClient.connected].connected = '';
    }
    thisClient.socket.close();
    delete clientWS[client];
};

const closeRobot = (client: string): void => {
    if (!(client in robotWS)) {
        console.error('tried to remove a non-existant robot');
        return;
    }
    const thisClient = robotWS[client];

    if (thisClient.connected in clientWS) {
        // If a robot disapears, we can't keep the client around
        closeClient(thisClient.connected);
    }
    thisClient.socket.close();
    delete robotWS[client];
};

// For connections from robots
robotWSserver.on(
    'connection',
    (ws: WebSocket, request: http.IncomingMessage, client: string) => {
        console.log('Robot ' + client + ' connected');
        if (client in robotWS) {
            closeRobot(client);
        }
        robotWS[client] = { socket: ws, connected: '', name: client };
        ws.on('message', (msg) => {
            if (robotWS[client].connected in clientWS) {
                clientWS[robotWS[client].connected].socket.send(msg);
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

        console.log(
            'Client ' + client + ' connected seeking robot: ' + targetRobot,
        );
        if (client in clientWS) {
            clientWS[client].socket.close();
        }
        if (!(targetRobot in robotWS)) {
            console.error('Non-existant target robot: ' + targetRobot);
            ws.close();
            return;
        }
        if (robotWS[targetRobot].connected !== '') {
            console.error('Tried to connected to an already taken robot');
            ws.close();
            return;
        } else {
            clientWS[client] = { socket: ws, connected: '', name: client };
            //TODO: this is a potential race condition
            robotWS[targetRobot].connected = client;
            clientWS[client].connected = targetRobot;
        }

        ws.on('message', (msg) => {
            robotWS[targetRobot].socket.send(msg);
        });
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
