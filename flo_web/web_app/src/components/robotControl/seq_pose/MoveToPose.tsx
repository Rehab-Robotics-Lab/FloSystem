import React, { useState } from "react";
import * as ROSLIB from "roslib";
import { SetMoving, JointState } from "../../robotController";
import { runSequence, Move } from "./SequenceRunContainer";
import {
  basicBlock,
  inputWithSpace,
  majorButton
} from "../../../styleDefs/styles";

const armNames = [
  "shoulder_flexionextension",
  "shoulder_abduction",
  "shoulder_rotation",
  "elbow_flexionextension"
];

interface ArmValProps {
  name: string;
  val: number;
  transfer: () => void;
}

const ArmVal: React.FunctionComponent<ArmValProps> = ({
  name,
  val,
  transfer
}) => {
  return (
    <div style={inputWithSpace}>
      <div>
        {name}: {val.toFixed(3)}
      </div>
      <div>
        <button type="button" onClick={(): void => transfer()}>
          Transfer
        </button>
      </div>
    </div>
  );
};

interface ArmInputProps {
  name: string;
  setTarget: (arg: number) => void;
  val: number;
  min?: number;
  max?: number;
}

const ArmInput: React.FunctionComponent<ArmInputProps> = ({
  name,
  setTarget,
  val,
  min = -180,
  max = 180
}) => {
  return (
    <div style={inputWithSpace}>
      <label htmlFor="arm_input">{name}:</label>
      <div>
        <input
          type="number"
          name="arm_input"
          style={{ width: "50px" }}
          min={min}
          max={max}
          step="any"
          value={val}
          onChange={(e): void => {
            setTarget(parseFloat(e.target.value));
          }}
        />
        <input
          type="range"
          min={min}
          max={max}
          value={val}
          onChange={(e): void => {
            setTarget(parseFloat(e.target.value));
          }}
        />
      </div>
    </div>
  );
};

interface MoveToPoseProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  moving: boolean;
  setMoving: SetMoving;
  pose: JointState | null;
}

const MoveToPose: React.FunctionComponent<MoveToPoseProps> = ({
  ros,
  connected,
  moving,
  setMoving,
  pose
}) => {
  const numArms = armNames.length;
  const [targetPose, setTargetPose] = useState(new Array(numArms).fill(0));
  const [arm, setArm] = useState<"left" | "right">("right");
  const [time, setTime] = useState(2);

  const inputs = [];
  for (let idx = 0; idx < numArms; idx += 1) {
    inputs.push(
      <ArmInput
        name={armNames[idx]}
        setTarget={(arg: number): void => {
          const targetPoseT = [...targetPose];
          targetPoseT[idx] = arg;
          setTargetPose(targetPoseT);
        }}
        val={targetPose[idx]}
        key={"input_" + armNames[idx]}
      />
    );
  }

  const currentPoses = [];
  if (pose) {
    for (const arm of ["right", "left"]) {
      for (let idx = 0; idx < numArms; idx += 1) {
        const target = pose.name.findIndex(
          p => p === arm + "_" + armNames[idx]
        );
        const degVal = (pose.position[target] * 180) / Math.PI;
        currentPoses.push(
          <ArmVal
            name={pose.name[target]}
            val={degVal}
            transfer={(): void => {
              const targetPoseT = [...targetPose];
              targetPoseT[idx] = degVal;
              setTargetPose(targetPoseT);
            }}
            key={"current_" + pose.name[target]}
          />
        );
      }
    }
  }

  return (
    <div
      style={Object.assign({}, basicBlock, {
        maxWidth: "none",
        maxHeight: "auto"
      })}
    >
      <h2>Move to a Pose</h2>
      <div style={{ display: "flex", flexWrap: "wrap", flexDirection: "row" }}>
        <div>
          <h3>Current Poses:</h3>
          {currentPoses}
        </div>
        <div>
          <h3>Target Poses</h3>
          Enter in degrees:
          <hr />
          {inputs}
          <hr />
          <label htmlFor="arm-selector">
            <button
              name="arm-selector"
              type="button"
              onClick={(): void => {
                setArm(arm === "left" ? "right" : "left");
              }}
            >
              {arm}
            </button>
          </label>
          <ArmInput name="time" setTarget={setTime} val={time} />
          <hr />
          <label htmlFor="run">
            <button
              name="run"
              type="button"
              style={majorButton}
              onClick={(): void => {
                if (ros === null) {
                  return;
                }

                const cleanNames = [];
                let name;
                for (name in armNames) {
                  cleanNames.push(arm + "_" + name);
                }

                const radPose = [];
                for (let idx = 0; idx < targetPose.length; idx += 1) {
                  radPose[idx] = (targetPose[idx] * Math.PI) / 180;
                }

                const movesList: Move[] = [
                  {
                    time: time,
                    pose: {
                      pose: {
                        description: "temp pose",
                        joint_names: armNames, // eslint-disable-line
                        joint_positions: radPose // eslint-disable-line
                      },
                      id: 0
                    },
                    lr: arm,
                    status: "not-run",
                    key: 0
                  }
                ];

                runSequence(movesList, () => null, setMoving, ros);
              }}
              disabled={moving || !connected}
            >
              Run
            </button>
          </label>
        </div>
      </div>
    </div>
  );
};

export default MoveToPose;
