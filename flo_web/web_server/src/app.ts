// lib/app.ts
import express from "express";
import http from "http";
import WebSocket from "ws";
import url from "url";
import bcrypt from "bcrypt";
import passport from "passport";
import session from "express-session";

const socketPort = 8080;
const expressPort = 3008;

// Create a new express application instance
const app: express.Application = express();

app.get("/", function (req, res) {
  res.send("Hello World!!!!");
});

app.listen(expressPort, "0.0.0.0", function () {
  console.log("Example app listening on port " + expressPort);
});

//const server = https.createServer({
//cert: fs.readFileSync('/path/to/cert.pem'),
//key: fs.readFileSync('/path/to/key.pem')
//});
const server = http.createServer();

const robotWSserver = new WebSocket.Server({ noServer: true });
const clientWSserver = new WebSocket.Server({ noServer: true });

let robotWS = null;
let clientWS = null;

robotWSserver.on("connection", (ws, request, client) => {
  clientWS = ws;
  console.log("client connecting for robot x");
  ws.on("message", (msg) => {
    robotWS.send(msg);
    console.log("received from robot: %s", msg);
  });
});

robotWSserver.on("listening", () => {
  console.log("robot websocket server listening");
});

clientWSserver.on("connection", (ws, request, client) => {
  console.log("robot y connected");
  robotWS = ws;
  ws.on("message", (msg) => {
    clientWS.send(msg);
    console.log("received from client: %s", msg);
  });
});

clientWSserver.on("listening", () => {
  console.log("robot websocket server listening");
});

server.on("upgrade", (request, socket, head) => {
  const path = url.parse(request.url).pathname;
  console.log(path);
  const pathSplits = path.split("/");
  const target = pathSplits[1];
  console.log("connection attempting to " + target);

  if (target === "host") {
    // For the robots to connect to
    robotWSserver.handleUpgrade(request, socket, head, (ws) => {
      robotWSserver.emit("connection", ws, request, "sue");
    });
  } else if (target === "robot") {
    // For the clients to hook up to a robot
    clientWSserver.handleUpgrade(request, socket, head, (ws) => {
      clientWSserver.emit("connection", ws, request, "bob");
    });
  } else {
    console.log("invalid websocket enpoint");
    socket.destroy();
  }
});

server.on("listening", () => {
  console.log(
    "Socket server listening at: " +
      server.address().address +
      ":" +
      server.address().port
  );
});

server.listen(socketPort, "0.0.0.0");
