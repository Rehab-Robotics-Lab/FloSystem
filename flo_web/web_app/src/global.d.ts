declare module "webrtc-adapter"{
    export type adapter= any;
}

declare module "ros3d" {
  import * as ROSLIB from "roslib";
  import * as THREE from "three";

  export class Viewer {
    constructor(options: {
      divID: string;
      width: number;
      height: number;
      background?: string;
      antialias?: boolean;
      intensity?: number;
      near?: number;
      far?: number;
      alpha?: number;
      cameraPose?: { x: number; y: number; z: number };
      cameraZoomSpeed?: number;
      displayPanAndZoomFrame?: boolean;
      lineTypePanAndZoomFrame?: string;
    });
    renderer: THREE.WebGLRenderer;
    scene: THREE.Scene;
    camera: THREE.PerspectiveCamera;
    cameraControls: object; //TODO: this should be a ROS3D.OrbitContros object
    directionalLight: Three.DirectionalLight;
    selectableObjects: THREE.Object3D;
    mouseHandler: object; //TODO: should be a ROS3D.MouseHandler
    stopped: boolean;
    animationRequestID: number; //TODO: Not sure what this is
    start(): void;
      draw():void;
      stop():void;
      addObject(obj: THREE.Object3D, selectable: boolean?):void;
      resize(width: number, height:number):void;
  }

  export class UrdfClient {
    constructor(options: {
      ros: ROSLIB.Ros;
      param?: string;
      path?: string;
      tfClient: ROSLIB.Ros.TFClient;
        rootObject?: object; //TODO: I have no clue what this is
      tfPrefix?: string;
      loader?: any;
    });
      urdf: ROS3D.Urdf;
  }

    export class Urdf{
        constructor(options:{
            urdfModel: ROSLIB.UrdfModel;
            path?:string;
            tfClient: ROSLIB.Ros.TFClient; 
            tfPrefix?: string;
            loader?: object; //TODO: not sure what type this is
        })
    }
}
