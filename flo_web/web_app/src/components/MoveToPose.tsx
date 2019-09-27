import React, { useState } from "react";
import * as ROSLIB from "roslib";
import { PoseMsg, PoseWrapper } from "./PoseContainer";
import { SetMoving, SetMovesList } from "../App";
import { runSequence } from "./SequenceRunContainer";

const armNames = [
  "left_shoulder_flexionextension",
  "shoulder_abduction",
  "shoulder_rotation",
  "elbow_flexionextension"
];

interface ArmInputProps {
  name: string;
  setTarget: (arg: number) => void;
  val: number;
}

const ArmInput: React.FunctionComponent<ArmInputProps> = ({
  name,
  setTarget,
  val
}) => {
  return (
    <div>
      <label htmlFor="arm_input">
        {name}:
        <input
          type="text"
          name="arm_input"
          value={val}
          onChange={e => {
            setTarget(parseFloat(e.target.value));
          }}
        />
      </label>
    </div>
  );
};

interface MoveToPoseProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  moving: boolean;
  setMoving: SetMoving;
}

const MoveToPose: React.FunctionComponent<MoveToPoseProps> = ({
  ros,
  connected,
  moving,
  setMoving
}) => {
  const numArms = armNames.length;
  const [targetPose, setTargetPose] = useState(new Array(numArms).fill(0));

  const inputs = [];
  for (let idx = 0; idx < numArms; idx += 1) {
    inputs.push(
      <ArmInput
        name={armNames[idx]}
        setTarget={(arg: number) => {
          const targetPoseT = [...targetPose];
          targetPoseT[idx] = arg;
          setTargetPose(targetPoseT);
        }}
        val={targetPose[idx]}
      />
    );
  }
  return (
    <div>
      <h2>Move to a Pose</h2>
      {inputs}
    </div>
  );
};

export default MoveToPose;
