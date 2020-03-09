import React, { useState, useEffect } from "react";
import * as ROSLIB from "roslib";
import { basicBlock } from "../styleDefs/styles";
import ModalWrapper from "./ModalWrapper";

interface StepDef {
  type: string;
  text: string;
  id: number;
  time: number;
}

interface GameAction {
  id: number;
  description: string;
}

interface GameBucketsProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}

interface GameActionOptProps {
  action: GameAction;
  setActiveOpt: (index: number) => void;
  active: boolean;
}

const GameActionOpt: React.FunctionComponent<GameActionOptProps> = ({
  action,
  setActiveOpt,
  active
}) => (
  <button
    type="button"
    onClick={(): void => {
      setActiveOpt(action.id);
    }}
    disabled={active}
  >
    {action.description}
  </button>
);

interface AddGameActionProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  addGameAction: (ga: StepDef) => void;
  cancel: () => void;
  showAdd: boolean;
}

enum ActionType {
  seq = "move",
  left_arm = "pose_left",
  right_arm = "pose_right",
  none = "none"
}

const AddGameAction: React.FunctionComponent<AddGameActionProps> = ({
  ros,
  addGameAction,
  cancel,
  showAdd,
  connected
}) => {
  const [gameActionOpts, setGameActionOpts] = useState<GameAction[]>([]);
  const [toSay, setToSay] = useState("");
  const [timeTarget, setTimeTarget] = useState(0);
  const [actionType, setActionType] = useState<ActionType>(ActionType.none);
  const [activeOpt, setActiveOpt] = useState(-1);
  return (
    <ModalWrapper show={showAdd}>
      <h1>Add Item</h1>
      <button
        type="button"
        onClick={(): void => {
          cancel();
        }}
      >
        Cancel
      </button>
      <div>
        <button
          type="button"
          disabled={!connected || actionType == ActionType.seq}
          onClick={(): void => {
            const searchSeqClient = new ROSLIB.Service({
              ros: ros as ROSLIB.Ros,
              name: "/search_pose_seq",
              serviceType: "flo_core/SearchPoseSeq"
            });
            console.log("connected to service to search for a pose sequence");

            const request = new ROSLIB.ServiceRequest({ search: "" });

            searchSeqClient.callService(request, resp => {
              const seqs = [];
              for (let i = 0; i < resp.ids.length; i += 1) {
                seqs.push({
                  id: resp.ids[i],
                  description: resp.sequences[i].description
                });
              }
              setGameActionOpts(seqs);
              console.log("received pose sequences");
            });
            console.log("searched for all pose sequences");
            setActionType(ActionType.seq);
          }}
        >
          Sequence
        </button>

        <label htmlFor="toSay">
          To Say:
          <input
            type="text"
            name="toSay"
            value={toSay}
            onChange={(e): void => setToSay(e.target.value)}
          />
        </label>
        <label htmlFor="timeTarget">
          Time (0 yields default):
          <input
            type="number"
            min="0"
            name="timeTarget"
            value={timeTarget}
            onChange={(e): void => setTimeTarget(Number(e.target.value))}
          />
        </label>
        <div>
          {gameActionOpts.map(value => (
            <GameActionOpt
              key={actionType + value.id}
              action={value}
              setActiveOpt={setActiveOpt}
              active={value.id == activeOpt}
            />
          ))}
        </div>
        <button
          type="button"
          disabled={
            actionType == ActionType.none ||
            timeTarget < 0 ||
            activeOpt < 0 ||
            activeOpt >= gameActionOpts.length
          }
          onClick={() => {
            addGameAction({
              type: actionType,
              text: toSay,
              id: activeOpt,
              time: timeTarget
            });
          }}
        >
          Add
        </button>
      </div>
    </ModalWrapper>
  );
};
// Takes a parameter ros, which is the connection to ros
const GameBuckets: React.FunctionComponent<GameBucketsProps> = ({
  ros,
  connected
}) => {
  const [steps, setSteps] = useState<StepDef[]>([]);
  const [showAdd, setShowAdd] = useState(false);
  return (
    <div style={basicBlock}>
      <h2>GameBuckets</h2>

      <button type="button">Load Bucket</button>
      <button type="button">Save Bucket</button>
      <button
        type="button"
        onClick={(): void => {
          setShowAdd(true);
        }}
      >
        Add Item
      </button>
      <hr />
      <AddGameAction
        ros={ros}
        addGameAction={(action: StepDef) => {
          setSteps([...steps, action]);
          setShowAdd(false);
        }}
        cancel={() => {
          setShowAdd(false);
        }}
        showAdd={showAdd}
        connected={connected}
      />
    </div>
  );
};

export default GameBuckets;
