import React, { useEffect, useState } from "react";
import * as ROSLIB from "roslib";

interface FaceState {
  left_eye: boolean[];
  right_eye: boolean[];
  eye_width: number;
  eye_height: number;
  eye_name: string;
  left_eye_brightness: number;
  right_eye_brightness: number;

  mouth: boolean[];
  mouth_width: number;
  mouth_height: number;
  mouth_name: string;
  mouth_description: string;
  mouth_brightness: number;
}

interface MatrixProps {
  x: number;
  y: number;
  height: number;
  width: number;
  on: boolean[];
  brightness: number;
  elementSize: number;
  pitch: number;
}

// Apparently there is a problem in the typings for returning
// an array from a function component:
// https://github.com/DefinitelyTyped/DefinitelyTyped/issues/20356#issuecomment-336384210
// best workaround is here:
// https://stackoverflow.com/questions/46709773/typescript-react-rendering-an-array-from-a-stateless-functional-component
const Matrix: React.FunctionComponent<MatrixProps> = ({
  x,
  y,
  height,
  width,
  on,
  brightness,
  elementSize,
  pitch,
}) => {
    const unitWidth = width*elementSize + (width-1)*pitch;
    const unitHeight = height*elementSize +(height-1)*pitch;
    const centerX = unitWidth/2;
    const centerY = unitHeight/2;
    const viewboxString = [0,0,unitWidth,unitHeight].join(" ");
  const components = [];
  for (let idx = 0; idx < on.length; idx += 1) {
    let row = Math.floor(idx / width);
    let col = idx % width;
    const xPos = col * (elementSize + pitch);
    const yPos = row * (elementSize + pitch);
    components.push(
      <rect
        x={xPos}
        y={yPos}
        width={elementSize}
        height={elementSize}
        display={on[idx] ? "" : "none"}
        opacity={brightness/15}
        fill="blue"
      />
    );
  }
  return <svg x={x-unitWidth/2} y={y-unitHeight/2} width={unitWidth} height={unitHeight} viewBox={viewboxString}>{components}</svg>;
};

interface CurrentFaceProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}
const CurrentFace: React.FunctionComponent<CurrentFaceProps> = ({
  ros,
  connected
}) => {
  const [faceState, setFaceState] = useState<FaceState | null>(null);
  const [faceListener, setFaceListener] = useState<ROSLIB.Topic | null>(null);

  useEffect(() => {
    if (!connected) return;
    // TODO: Figure out how to clean up pose listener
    // poseListener.unsubscribe();
    // setPoseListener(null);
    const faceListenerT = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "face_state",
      messageType: "flo_face/FaceState"
    });
    faceListenerT.subscribe(msg => {
      setFaceState(msg as FaceState);
    });
    setFaceListener(faceListenerT);
  }, [connected, ros]);

    const faceMatrices = []
    if(faceState){
        const eyeSep = (faceState.eye_width*3 +35)/2;
        const vertSep = ((faceState.eye_height/2 +faceState.mouth_height/2)*3 +15)/2;
        faceMatrices.push([
          <Matrix
            x={eyeSep}
            y={-vertSep}
            height={faceState.eye_height}
            width={faceState.eye_width}
            on={faceState.right_eye}
            brightness={faceState.right_eye_brightness}
            elementSize={2}
            pitch={1}
        />,
          <Matrix
            x={-eyeSep}
            y={-vertSep}
            height={faceState.eye_height}
            width={faceState.eye_width}
            on={faceState.left_eye}
            brightness={faceState.left_eye_brightness}
            elementSize={2}
            pitch={1}
        />,
          <Matrix
            x={0}
            y={vertSep}
            height={faceState.mouth_height}
            width={faceState.mouth_width}
            on={faceState.mouth}
            brightness={faceState.mouth_brightness}
            elementSize={2}
            pitch={1}
          />
   ])
    }
  return (
    <div>
      <h3>Current Face :</h3>
      <svg width="100%"  viewBox="-45 -35 90 70">
          {faceMatrices}
      </svg>
    </div>
  );
};

export default CurrentFace;
