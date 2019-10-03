import React, { useEffect, useState } from "react";
import * as ROSLIB from "roslib";
import { SetSpeechTarget, Utterance } from "../App";
import { basicBlock } from "../styleDefs/styles";
import SavedFaces from "./SavedFaces";

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

  return (
    <div
      style={Object.assign({}, basicBlock, {
        maxWidth: "300px"
      })}
    >
      <h2>Faces:</h2>
      <SavedFaces
        ros={ros}
        connected={connected}
        setEyeOptions={setEyeOptions}
      />
    </div>
  );
};

export default FaceContainer;
