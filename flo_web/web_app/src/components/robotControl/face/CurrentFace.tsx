import React, { useEffect } from "react";
import * as ROSLIB from "roslib";
import { FaceState } from "./FaceContainer";

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

const Matrix: React.FunctionComponent<MatrixProps> = ({
  x,
  y,
  height,
  width,
  on,
  brightness,
  elementSize,
  pitch
}) => {
  const unitWidth = width * elementSize + (width - 1) * pitch;
  const unitHeight = height * elementSize + (height - 1) * pitch;
  const viewboxString = [0, 0, unitWidth, unitHeight].join(" ");
  const components = [];
  for (let idx = 0; idx < on.length; idx += 1) {
    const row = Math.floor(idx / width);
    const col = idx % width;
    const xPos = col * (elementSize + pitch);
    const yPos = row * (elementSize + pitch);
    components.push(
      <rect
        x={xPos}
        y={yPos}
        width={elementSize}
        height={elementSize}
        display={on[idx] ? "" : "none"}
        opacity={brightness / 15}
        fill="blue"
      />
    );
  }
  return (
    <svg
      x={x - unitWidth / 2}
      y={y - unitHeight / 2}
      width={unitWidth}
      height={unitHeight}
      viewBox={viewboxString}
    >
      {components}
    </svg>
  );
};

interface CurrentFaceProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  faceState: FaceState | null;
  setFaceState: (state: FaceState) => void;
}
const CurrentFace: React.FunctionComponent<CurrentFaceProps> = ({
  ros,
  connected,
  faceState,
  setFaceState
}) => {
  useEffect(() => {
    if (!connected) {
      console.log("Not subscribing to topic because not connected");
      return;
    }
    const faceListenerT = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "face_state",
      messageType: "flo_face_defs/FaceState"
    });
    const callback = (msg: ROSLIB.Message): void => {
      setFaceState(msg as FaceState);
    };
    faceListenerT.subscribe(callback);
    console.log("subscribed to face_state");

    return (): void => {
      faceListenerT.unsubscribe(callback);
    };
  }, [connected, ros, setFaceState]);

  const faceMatrices = [];
  if (faceState) {
    const eyeSep = (faceState.eye_width * 3 + 35) / 2;
    const vertSep =
      ((faceState.eye_height / 2 + faceState.mouth_height / 2) * 3 + 15) / 2;
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
        key={1}
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
        key={2}
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
        key={3}
      />
    ]);
  }
  return (
    <div>
      <h3>Current Face :</h3>
      <svg width="100%" viewBox="-45 -35 90 70">
        {faceMatrices}
      </svg>
    </div>
  );
};

export default CurrentFace;
