import React, { useState } from "react";
import * as ROSLIB from "roslib";
import { PoseWrapper } from "./PoseContainer";
import { SetMoving, SetMovesList } from "../../robotController";
import {
  basicBlock,
  majorButton,
  buttonContainer
} from "../../../styleDefs/styles";

interface SetTime {
  (id: number, time: number): void;
}

interface SimpleModificationFunc {
  (id: number): void;
}

interface MoveProps {
  id: number;
  pose: PoseWrapper;
  time: number;
  lr: "left" | "right";
  setTime: SetTime;
  toggleLR: SimpleModificationFunc;
  moveUp: SimpleModificationFunc;
  moveDown: SimpleModificationFunc;
  remove: SimpleModificationFunc;
  status: string;
  prior_lr: "left" | "right" | "";
  priorTime: number;
  setErrorState: (arg: boolean) => void;
}

export interface Move {
  time: number;
  pose: PoseWrapper;
  lr: "left" | "right";
  status: "not-run" | "complete" | "moving" | "none" | "planned";
  key: number;
}

export const runSequence = (
  MovesList: Move[],
  setMovesList: SetMovesList,
  setMoving: SetMoving,
  ros: ROSLIB.Ros
): void => {
  if (ros === null) {
    return;
  }
  const actionClient = new ROSLIB.ActionClient({
    ros: ros,
    serverName: "/move",
    actionName: "flo_humanoid_defs/MoveAction",
    timeout: 1500 //Not sure about this value here. needs testing
  });
  console.log("connected to move action action client");

  let time = 0;

  // Need to iterate through the move list and do:
  // - add limb prefix
  // - calculate the actual target time
  const targets: Array<ROSLIB.Message> = [];
  let priorArm = "";
  MovesList.forEach(move => {
    if (move.lr === priorArm && move.time === 0) {
      throw new Error(
        "There cannot be two arm movements with zero time on the same arm"
      );
    }
    priorArm = move.lr;
    time += move.time;
    const names = [...move.pose.pose.joint_names];
    for (let idx = 0; idx < names.length; idx += 1) {
      names[idx] = `${move.lr}_${names[idx]}`;
    }
    const msg = new ROSLIB.Message({
      name: names,
      position: move.pose.pose.joint_positions,
      target_completion_time: time // eslint-disable-line
    });
    targets.push(msg);
  });

  const goal = new ROSLIB.Goal({
    actionClient,
    goalMessage: {
      targets
    }
  });

  //TODO: get this out as an error
  (actionClient as any).on("timeout", () => {
    alert("failed to attach to move server due to timeout, you can try again");
    setMoving(false);
    const moveListN = [...MovesList];
    for (let idx = 0; idx < moveListN.length; idx += 1) {
      moveListN[idx].status = "none";
    }
    setMovesList(moveListN);
    goal.cancel();
  });

  goal.on("timeout", () => {
    alert("failed to move due to timeout, you can try again");
    setMoving(false);
    const moveListN = [...MovesList];
    for (let idx = 0; idx < moveListN.length; idx += 1) {
      moveListN[idx].status = "none";
    }
    setMovesList(moveListN);
    goal.cancel();
  });

  goal.on("feedback", fb => {
    const curMove = fb.move_number;
    const moveListN = [...MovesList];
    for (let idx = 0; idx < curMove; idx += 1) {
      moveListN[idx].status = "complete";
    }
    moveListN[curMove].status = "moving";
    setMovesList(moveListN);
    console.log("got move action feedback");
  });

  goal.on("result", res => {
    setMoving(false);
    const moveListN = [...MovesList];
    for (let idx = 0; idx < moveListN.length; idx += 1) {
      moveListN[idx].status = "none";
    }
    setMovesList(moveListN);
    if (res.completed === false) {
      //TODO set an error
      console.error("move is done but did not finish");
    }
    console.log("movement done");
  });

  setMoving(true);
  const moveListN = [...MovesList];
  for (let idx = 0; idx < moveListN.length; idx += 1) {
    moveListN[idx].status = "planned";
  }
  setMovesList(moveListN);

  goal.send(time * 1000 * 2);
  console.log("sent command to move");
};

const Move: React.FunctionComponent<MoveProps> = ({
  id,
  pose,
  lr,
  time,
  setTime,
  toggleLR,
  moveUp,
  moveDown,
  remove,
  status,
  prior_lr,
  priorTime,
  setErrorState
}) => {
  const style: React.CSSProperties = {
    display: "flex",
    flexDirection: "row",
    flexWrap: "nowrap",
    background: id % 2 ? "#bcd2e0" : "white",
    border: "none"
  };
  if (status === "complete") {
    style.border = "2px solid green";
  } else if (status === "moving") {
    style.border = "2px solid blue";
  }

  let errorTitle = "";

  if (time <= 0 && prior_lr === lr) {
    errorTitle = "You cannot have zero time between two moves on the same arm";
    setErrorState(true);
  } else if (time === 0 && priorTime === 0) {
    errorTitle = "You can't have zero time for two consecutive moves";
    setErrorState(true);
  } else {
    setErrorState(false);
  }
  return (
    <div style={{ display: "flex", flexDirection: "column" }}>
      <div style={style}>
        <span style={{ width: "100px", flexShrink: 0 }}>
          {pose.pose.description}
        </span>
        <input
          type="number"
          min="0"
          max="10"
          step="any"
          value={time}
          onChange={(e): void => setTime(id, parseFloat(e.target.value))}
          style={{ width: "35px" }}
        />
        <button
          type="button"
          onClick={(): void => {
            toggleLR(id);
          }}
        >
          {lr}
        </button>
        <button
          type="button"
          onClick={(): void => {
            moveUp(id);
          }}
        >
          &uarr;
        </button>
        <button
          type="button"
          onClick={(): void => {
            moveDown(id);
          }}
        >
          &darr;
        </button>
        <button
          type="button"
          onClick={(): void => {
            remove(id);
          }}
        >
          &#10007;
        </button>
      </div>
      {errorTitle && <span style={{ color: "red" }}>{errorTitle}</span>}
    </div>
  );
};

interface SequenceRunContainerProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  MovesList: Move[];
  setMovesList: SetMovesList;
  moving: boolean;
  setMoving: SetMoving;
}

// Takes a parameter ros, which is the connection to ros
const SequenceRunContainer: React.FunctionComponent<SequenceRunContainerProps> = ({
  ros,
  connected,
  MovesList,
  setMovesList,
  moving,
  setMoving
}) => {
  const [errorList, setErrorList] = useState<boolean[]>([]);
  const setTime: SetTime = (id, time) => {
    const moveListN = [...MovesList];
    const target = moveListN[id];
    target.time = time;
    moveListN[id] = target;
    setMovesList(moveListN);
  };

  const toggleLR: SimpleModificationFunc = id => {
    const moveListN = [...MovesList];
    const target = moveListN[id];
    target.lr = target.lr === "left" ? "right" : "left";
    moveListN[id] = target;
    setMovesList(moveListN);
  };

  const moveUp: SimpleModificationFunc = id => {
    if (id === 0) return;
    const moveListN = [...MovesList];
    moveListN.splice(id, 0, moveListN.splice(id - 1, 1)[0]);
    setMovesList(moveListN);
  };

  const moveDown: SimpleModificationFunc = id => {
    if (id === MovesList.length - 1) return;
    const moveListN = [...MovesList];
    moveListN.splice(id, 0, moveListN.splice(id + 1, 1)[0]);
    setMovesList(moveListN);
  };

  const remove: SimpleModificationFunc = id => {
    const moveListN = [...MovesList];
    moveListN.splice(id, 1);
    setMovesList(moveListN);
  };

  // need to check that two successive commands don't have the same limb with zero time.

  const renderedMoves = [];
  for (let index = 0; index < MovesList.length; index += 1) {
    let value = MovesList[index];
    if (value !== null && value !== undefined) {
      renderedMoves.push(
        <Move
          id={index}
          key={value.key}
          pose={value.pose}
          lr={value.lr}
          toggleLR={toggleLR}
          time={value.time}
          setTime={setTime}
          moveUp={moveUp}
          moveDown={moveDown}
          remove={remove}
          status={value.status}
          prior_lr={
            index === 0 || MovesList[index - 1] === undefined
              ? ""
              : MovesList[index - 1].lr
          }
          priorTime={
            index === 0 || MovesList[index - 1] === undefined
              ? 0
              : MovesList[index - 1].time
          }
          setErrorState={val => {
            if (val !== errorList[index]) {
              const errorListT: boolean[] = [...errorList];
              errorListT[index] = val;
              setErrorList(errorListT);
            }
          }}
        />
      );
    }
  }

  return (
    <div
      id="moves-container"
      style={Object.assign({}, basicBlock, {
        flexShrink: 0
      })}
    >
      <h2>List of moves to make:</h2>
      <div
        style={{
          display: "flex",
          flexDirection: "column",
          overflowY: "auto",
          overflowX: "hidden"
        }}
      >
        <div style={{ display: "flex", flexDirection: "row" }}>
          <div>Pose</div>
          <div>Time from prior (s)</div>
          <div>Arm</div>
          <div>Up</div>
          <div>Down</div>
          <div>Delete</div>
        </div>
        {renderedMoves}
      </div>
      <hr />
      <div style={buttonContainer}>
        <button
          type="button"
          style={majorButton}
          disabled={!connected || moving || errorList.some(val => val)}
          onClick={(): void => {
            if (ros === null) {
              return;
            }
            runSequence(MovesList, setMovesList, setMoving, ros);
          }}
        >
          Run Sequence
        </button>
        <button
          type="button"
          onClick={(): void => {
            setMovesList([]);
          }}
        >
          Clear
        </button>
      </div>
    </div>
  );
};

export default SequenceRunContainer;
