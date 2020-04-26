import React, { useState } from "react";
import * as ROSLIB from "roslib";
import { basicBlock } from "../../../styleDefs/styles";
import SavedFaces from "./SavedFaces";
import CurrentFace from "./CurrentFace";
import EyeOptionsContainer from "./EyeOptionsContainer";
import FaceBrightnessSet from "./FaceBrightnessSet";

export interface FaceState {
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
export interface SetEyeOptions {
  (arg: string[]): void;
}

interface FaceContainerProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}
// Takes a parameter ros, which is the connection to ros
const FaceContainer: React.FunctionComponent<FaceContainerProps> = ({
  ros,
  connected
}) => {
  const [eyeOptions, setEyeOptions] = useState<string[]>([]);
  const [faceState, setFaceState] = useState<FaceState | null>(null);

  return (
    <div
      style={Object.assign({}, basicBlock, {
        maxwidth: "400px"
      })}
    >
      <h2>Faces:</h2>
      <div
        style={{
          display: "flex",
          flexDirection: "row",
          overflow: "hidden",
          flex: "1"
        }}
      >
        <SavedFaces
          ros={ros}
          connected={connected}
          setEyeOptions={setEyeOptions}
        />
        <div
          style={{
            maxWidth: "200px",
            overflow: "hidden",
            display: "flex",
            flexDirection: "column"
          }}
        >
          <CurrentFace
            ros={ros}
            connected={connected}
            setFaceState={setFaceState}
            faceState={faceState}
          />
          <FaceBrightnessSet
            ros={ros}
            connected={connected}
            faceState={faceState}
          />
          <EyeOptionsContainer
            ros={ros}
            connected={connected}
            eyeOptions={eyeOptions}
          />
        </div>
      </div>
    </div>
  );
};

export default FaceContainer;
