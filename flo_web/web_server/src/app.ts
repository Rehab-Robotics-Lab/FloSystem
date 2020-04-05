// lib/app.ts
import express = require("express");

const robotWSport = 8089;
const expressPort = 3008;

// Create a new express application instance
const app: express.Application = express();

app.get("/", function (req, res) {
  res.send("Hello World!!!!");
});

app.listen(expressPort, function () {
  console.log("Example app listening on port " + expressPort);
});

const WebSocketServer = require("websocket").server;
const http = require("http");

const robotServer = http.createServer((request, response) => {
  console.log(new Date() + "Received req for " + request.url);
  response.writeHead(404);
  response.end();
});

robotServer.listen(robotWSport, () => {
  console.log(
    new Date() + " Robot WS Server is listening on port " + robotWSport
  );
});

const robotWSserver = new WebSocketServer({
  httpServer: robotServer,
  autoAcceptConnections: false,
});

function originIsAllowed(origin) {
  //TODO: add something here to see if we want to hear from this robot
  return true;
}

wsServer.on("request", (request) => {
  if (!originIsAllowed(request.origin)) {
    request.reject();
    console.log(
      new Date() + " connection from origin " + request.orgin + " rejected."
    );
    return;
  }

  const connection = request.accept();
});
