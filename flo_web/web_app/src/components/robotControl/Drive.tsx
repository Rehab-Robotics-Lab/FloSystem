import React, { useEffect } from "react";
import { basicBlock } from "../../styleDefs/styles";
import * as ROSLIB from "roslib";

interface Size {
  width: number;
  height: number;
}

interface Pos {
  x: number;
  y: number;
}

class Mouse {
  pos: Pos;
  clicked: boolean;
  readonly posNormal: Pos;

  constructor() {
    this.pos = { x: 0, y: 0 };
    this.posNormal = { x: 0, y: 0 };
    this.clicked = false;
  }

  normalize(center: Pos, factor: number): void {
    this.posNormal.x = (this.pos.x - center.x) * factor;
    this.posNormal.y = -(this.pos.y - center.y) * factor;
  }
}

interface ActFunc {
  (mouse: Mouse): void;
}

//let print2console: ActFunc = function(mouse: Mouse) {
//let message: string =
//"Mouse position: " + mouse.pos_normal.x + "," + mouse.pos_normal.y;
//console.log(message);
//};

//class WebSocketConnection{
//private socket;

//constructor(address: string){
//this.socket = new WebSocket(address);
//}

//communicate(mouse: Mouse){
//let msg = JSON.stringify(mouse);
//this.socket.send(msg);
//}
//}

class Joystick {
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private canvasSize: Size;
  private mouse: Mouse;
  private actFunc: ActFunc;

  private boundMouseMove: (ev: MouseEvent) => void;
  private boundMouseDown: (ev: MouseEvent) => void;
  private boundMouseOut: (ev: MouseEvent) => void;

  private boundTouchMove: (ev: TouchEvent) => void;
  private boundTouchDown: (ev: TouchEvent) => void;
  private boundTouchOut: (ev: TouchEvent) => void;

  private xPivot: number;
  private yPivot: number;

  private circleCenter: Pos;
  private circleRadius: number;

  constructor(canvas: HTMLCanvasElement, actFunc: ActFunc) {
    this.canvas = canvas;
    const context2d = this.canvas.getContext("2d");
    this.ctx = context2d as CanvasRenderingContext2D;
    this.mouse = new Mouse();
    this.actFunc = actFunc;
    this.canvasSize = { width: this.canvas.width, height: this.canvas.height };
    this.circleCenter = {
      x: this.canvasSize.width / 2,
      y: this.canvasSize.height / 2
    };
    this.circleRadius =
      Math.max(this.canvasSize.height, this.canvasSize.width) * 0.4;
    this.xPivot = this.circleRadius * 0.96824583655;
    this.yPivot = this.circleRadius / 4;

    this.boundMouseMove = this.mouseMove.bind(this);
    this.boundMouseDown = this.mouseDown.bind(this);
    this.boundTouchDown = this.touchDown.bind(this);
    this.boundMouseOut = this.mouseOut.bind(this);
    this.boundTouchOut = this.touchOut.bind(this);
    this.boundTouchMove = this.touchMove.bind(this);

    this.drawBackground();

    canvas.addEventListener("mousedown", this.boundMouseDown, false);
    canvas.addEventListener("touchstart", this.boundTouchDown, false);
  }

  getMousePos(evt: MouseEvent): { x: number; y: number } {
    const rect = this.canvas.getBoundingClientRect();
    return {
      x: evt.clientX - rect.left,
      y: evt.clientY - rect.top
    };
  }

  drawBackground(): void {
    this.ctx.clearRect(0, 0, this.canvasSize.width, this.canvasSize.height);

    this.ctx.beginPath();
    this.ctx.fillStyle = "#c5cedd";
    this.ctx.fillRect(0, 0, this.canvasSize.width, this.canvasSize.height);
    this.ctx.stroke();

    this.ctx.beginPath();
    this.ctx.strokeStyle = "#a3acba";
    this.ctx.lineWidth = 1;
    this.ctx.moveTo(
      this.circleCenter.x - this.xPivot,
      this.circleCenter.y + this.yPivot
    );
    this.ctx.lineTo(
      this.circleCenter.x + this.xPivot,
      this.circleCenter.y + this.yPivot
    );
    this.ctx.moveTo(
      this.circleCenter.x - this.xPivot,
      this.circleCenter.y - this.yPivot
    );
    this.ctx.lineTo(
      this.circleCenter.x + this.xPivot,
      this.circleCenter.y - this.yPivot
    );
    this.ctx.stroke();

    this.ctx.beginPath();
    this.ctx.strokeStyle = "#3d5d91";
    this.ctx.lineWidth = 5;
    this.ctx.arc(
      this.circleCenter.x,
      this.circleCenter.y,
      this.circleRadius,
      0,
      2 * Math.PI
    );
    this.ctx.stroke();
  }

  actOnInput(position: Pos): void {
    this.mouse.pos = position;
    this.mouse.clicked = true;
    this.drawBackground();
    if (this.mouse.clicked) {
      this.ctx.beginPath();
      this.ctx.strokeStyle = "#46dbbd";
      this.ctx.lineWidth = 3;
      this.ctx.moveTo(this.circleCenter.x, this.circleCenter.y);
      this.ctx.lineTo(this.mouse.pos.x, this.mouse.pos.y);
      this.ctx.stroke();
    }
    this.sendData();
  }

  sendData(): void {
    this.mouse.normalize(this.circleCenter, 1 / this.circleRadius);
    this.actFunc(this.mouse);
  }

  actOnMouse(evt: MouseEvent): void {
    const mousePos: Pos = this.getMousePos(evt);
    this.actOnInput(mousePos);
  }

  mouseMove(evt: MouseEvent): void {
    this.actOnMouse(evt);
  }

  mouseOut(): void {
    this.mouse.clicked = false;
    this.sendData();
    this.drawBackground();
    this.canvas.removeEventListener("mousemove", this.boundMouseMove, false);
    this.canvas.removeEventListener("mouseout", this.boundMouseOut, false);
    this.canvas.removeEventListener("mouseup", this.boundMouseOut, false);
  }

  mouseDown(evt: MouseEvent): void {
    this.actOnMouse(evt);
    this.canvas.addEventListener("mousemove", this.boundMouseMove, false);
    this.canvas.addEventListener("mouseout", this.boundMouseOut, false);
    this.canvas.addEventListener("mouseup", this.boundMouseOut, false);
  }

  touchOut(): void {
    this.mouse.clicked = false;
    this.sendData();
    this.drawBackground();
    this.canvas.removeEventListener("touchend", this.boundTouchOut, false);
    this.canvas.removeEventListener("touchcancel", this.boundTouchOut, false);
    this.canvas.removeEventListener("touchmove", this.boundTouchMove, false);
  }

  touchMove(evt: TouchEvent): void {
    const rect = this.canvas.getBoundingClientRect();
    const touch: Touch = evt.targetTouches.item(0) as Touch;
    this.actOnInput({
      x: touch.clientX - rect.left,
      y: touch.clientY - rect.top
    });
  }

  touchDown(evt: TouchEvent): void {
    evt.preventDefault(); //Prevents other touch functions like dragging from kicking in
    this.touchMove(evt);
    this.canvas.addEventListener("touchend", this.boundTouchOut, false);
    this.canvas.addEventListener("touchcancel", this.boundTouchOut, false);
    this.canvas.addEventListener("touchmove", this.boundTouchMove, false);
  }
}

//let canvas: HTMLCanvasElement = <HTMLCanvasElement>document.getElementById('joystick1');
// let connection = new WebSocketConnection('ws://127.0.0.1:5678'); //Localhost
//let connection = new WebSocketConnection('ws://10.10.15.78:5678');
//let joystick1 = new Joystick(canvas, connection.communicate.bind(connection));

interface DriveProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}

// Takes a parameter ros, which is the connection to ros
const Drive: React.FunctionComponent<DriveProps> = ({ ros, connected }) => {
  const canvasRef = React.useRef(null);
  const timer = React.useRef<number | undefined>(undefined);

  useEffect(() => {
    if (!connected || !ros) {
      return;
    }

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: "/keyop_vel_smoother/raw_cmd_vel",
      messageType: "geometry_msgs/Twist"
    });
    console.log("connected to raw command velocity topic");

    const publishFunc = (mouse: Mouse): void => {
      let twist = 0;
      let linear = 0;

      const sendVals = (linear: number, twist: number): void => {
        window.clearTimeout(timer.current);
        timer.current = undefined;
        const linearMsg = new ROSLIB.Message({
          x: linear,
          y: 0,
          z: 0
        });

        const angMsg = new ROSLIB.Message({
          x: 0,
          y: 0,
          z: twist
        });

        const msg = new ROSLIB.Message({
          linear: linearMsg,
          angular: angMsg
        });

        // trying to publish at 10hz
        topic.publish(msg);
        timer.current = window.setTimeout(sendVals, 100, linear, twist);
        //console.log("published target velocity: ", linear, twist); this publishes at 10 Hz, clobers terminal
      };

      if (mouse.clicked) {
        twist = -mouse.posNormal.x;
        linear = mouse.posNormal.y / 2;
        sendVals(linear, twist);
      } else {
        sendVals(0, 0);
      }
    };

    const canvas = (canvasRef.current as unknown) as HTMLCanvasElement;
    const joystick1 = new Joystick(canvas, publishFunc); //eslint-disable-line

    return (): void => {
      topic.unadvertise();
    };
  }, [ros, connected]);

  return (
    <div style={basicBlock}>
      <h2>Drive Robot</h2>
      <canvas
        style={{ borderRadius: "5%" }}
        ref={canvasRef}
        id="joystick1"
        width="300"
        height="300"
      ></canvas>
    </div>
  );
};

export default Drive;
