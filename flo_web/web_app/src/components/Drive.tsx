import React, { useEffect, useState } from "react";
import { basicBlock } from "../styleDefs/styles";
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
  readonly pos_normal: Pos;

  constructor() {
    this.pos = { x: 0, y: 0 };
    this.pos_normal = { x: 0, y: 0 };
    this.clicked = false;
  }

  normalize(center: Pos, factor: number) {
    this.pos_normal.x = (this.pos.x - center.x) * factor;
    this.pos_normal.y = -(this.pos.y - center.y) * factor;
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
  private canvas_size: Size;
  private mouse: Mouse;
  private act_func: ActFunc;

  private bound_mouse_move: (ev: MouseEvent) => void;
  private bound_mouse_down: (ev: MouseEvent) => void;
  private bound_mouse_out: (ev: MouseEvent) => void;

  private bound_touch_move: (ev: TouchEvent) => void;
  private bound_touch_down: (ev: TouchEvent) => void;
  private bound_touch_out: (ev: TouchEvent) => void;

  private x_pivot: number;
  private y_pivot: number;

  private circle_center: Pos;
  private circle_radius: number;

  constructor(canvas: HTMLCanvasElement, act_func: ActFunc) {
    this.canvas = canvas;
    let context2d = this.canvas.getContext("2d");
    this.ctx = context2d as CanvasRenderingContext2D;
    this.mouse = new Mouse();
    this.act_func = act_func;
    this.canvas_size = { width: this.canvas.width, height: this.canvas.height };
    this.circle_center = {
      x: this.canvas_size.width / 2,
      y: this.canvas_size.height / 2
    };
    this.circle_radius =
      Math.max(this.canvas_size.height, this.canvas_size.width) * 0.4;
    this.x_pivot = this.circle_radius * 0.96824583655;
    this.y_pivot = this.circle_radius / 4;

    this.bound_mouse_move = this.mouse_move.bind(this);
    this.bound_mouse_down = this.mouse_down.bind(this);
    this.bound_touch_down = this.touch_down.bind(this);
    this.bound_mouse_out = this.mouse_out.bind(this);
    this.bound_touch_out = this.touch_out.bind(this);
    this.bound_touch_move = this.touch_move.bind(this);

    this.draw_background();

    canvas.addEventListener("mousedown", this.bound_mouse_down, false);
    canvas.addEventListener("touchstart", this.bound_touch_down, false);
  }

  get_mouse_pos(evt: MouseEvent) {
    let rect = this.canvas.getBoundingClientRect();
    return {
      x: evt.clientX - rect.left,
      y: evt.clientY - rect.top
    };
  }

  draw_background() {
    this.ctx.clearRect(0, 0, this.canvas_size.width, this.canvas_size.height);

    this.ctx.beginPath();
    this.ctx.fillStyle = "#c5cedd";
    this.ctx.fillRect(0, 0, this.canvas_size.width, this.canvas_size.height);
    this.ctx.stroke();

    this.ctx.beginPath();
    this.ctx.strokeStyle = "#a3acba";
    this.ctx.lineWidth = 1;
    this.ctx.moveTo(
      this.circle_center.x - this.x_pivot,
      this.circle_center.y + this.y_pivot
    );
    this.ctx.lineTo(
      this.circle_center.x + this.x_pivot,
      this.circle_center.y + this.y_pivot
    );
    this.ctx.moveTo(
      this.circle_center.x - this.x_pivot,
      this.circle_center.y - this.y_pivot
    );
    this.ctx.lineTo(
      this.circle_center.x + this.x_pivot,
      this.circle_center.y - this.y_pivot
    );
    this.ctx.stroke();

    this.ctx.beginPath();
    this.ctx.strokeStyle = "#3d5d91";
    this.ctx.lineWidth = 5;
    this.ctx.arc(
      this.circle_center.x,
      this.circle_center.y,
      this.circle_radius,
      0,
      2 * Math.PI
    );
    this.ctx.stroke();
  }

  act_on_input(position: Pos) {
    this.mouse.pos = position;
    this.mouse.clicked = true;
    this.draw_background();
    if (this.mouse.clicked) {
      this.ctx.beginPath();
      this.ctx.strokeStyle = "#46dbbd";
      this.ctx.lineWidth = 3;
      this.ctx.moveTo(this.circle_center.x, this.circle_center.y);
      this.ctx.lineTo(this.mouse.pos.x, this.mouse.pos.y);
      this.ctx.stroke();
    }
    this.send_data();
  }

  send_data() {
    this.mouse.normalize(this.circle_center, 1 / this.circle_radius);
    this.act_func(this.mouse);
  }

  act_on_mouse(evt: MouseEvent) {
    let mouse_pos: Pos = this.get_mouse_pos(evt);
    this.act_on_input(mouse_pos);
  }

  mouse_move(evt: MouseEvent) {
    this.act_on_mouse(evt);
  }

  mouse_out(evt: MouseEvent) {
    this.mouse.clicked = false;
    this.send_data();
    this.draw_background();
    this.canvas.removeEventListener("mousemove", this.bound_mouse_move, false);
    this.canvas.removeEventListener("mouseout", this.bound_mouse_out, false);
    this.canvas.removeEventListener("mouseup", this.bound_mouse_out, false);
  }

  mouse_down(evt: MouseEvent) {
    this.act_on_mouse(evt);
    this.canvas.addEventListener("mousemove", this.bound_mouse_move, false);
    this.canvas.addEventListener("mouseout", this.bound_mouse_out, false);
    this.canvas.addEventListener("mouseup", this.bound_mouse_out, false);
  }

  touch_out(evt: TouchEvent) {
    this.mouse.clicked = false;
    this.send_data();
    this.draw_background();
    this.canvas.removeEventListener("touchend", this.bound_touch_out, false);
    this.canvas.removeEventListener("touchcancel", this.bound_touch_out, false);
    this.canvas.removeEventListener("touchmove", this.bound_touch_move, false);
  }

  touch_move(evt: TouchEvent) {
    let rect = this.canvas.getBoundingClientRect();
    let touch: Touch = evt.targetTouches.item(0) as Touch;
    this.act_on_input({
      x: touch.clientX - rect.left,
      y: touch.clientY - rect.top
    });
  }

  touch_down(evt: TouchEvent) {
    evt.preventDefault(); //Prevents other touch functions like dragging from kicking in
    this.touch_move(evt);
    this.canvas.addEventListener("touchend", this.bound_touch_out, false);
    this.canvas.addEventListener("touchcancel", this.bound_touch_out, false);
    this.canvas.addEventListener("touchmove", this.bound_touch_move, false);
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

  useEffect(() => {
    if (!connected || !ros) {
      return;
    }

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: "/keyop_vel_smoother/raw_cmd_vel",
      messageType: "geometry_msgs/Twist"
    });

    const publishFunc = (mouse: Mouse) => {
      let twist = 0;
      let linear = 0;

      const sendVals = (linear: number, twist: number) => {
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
      };

      if (mouse.clicked) {
        twist = mouse.pos_normal.x / 5;
        linear = mouse.pos_normal.y / 2;
        sendVals(linear, twist);
      }
    };

    const canvas = (canvasRef.current as unknown) as HTMLCanvasElement;
    let joystick1 = new Joystick(canvas, publishFunc);
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
