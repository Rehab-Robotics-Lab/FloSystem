import React, { useState } from "react";
import * as ROSLIB from "roslib";
import { basicBlock } from "../../styleDefs/styles";
import ModalWrapper from "./ModalWrapper";

interface StepDef {
  type: string;
  text: string;
  id: number;
  time: number;
  desc: string;
  key: string;
}

interface GameAction {
  id: number;
  description: string;
}

interface GameBucketsProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}

interface GameStepProps {
  def: StepDef;
  del: () => void;
}

const GameStep: React.FunctionComponent<GameStepProps> = ({ def, del }) => {
  return (
    <div style={{ display: "flex", justifyContent: "space-between" }}>
      <div>{def.type + ": " + def.desc}</div>
      <div>{`(${def.time}s) "${def.text}"`}</div>
      <button
        type="button"
        onClick={(): void => {
          del();
        }}
      >
        X
      </button>
    </div>
  );
};

interface GameActionOptProps {
  action: GameAction;
  setActiveOpt: () => void;
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
      setActiveOpt();
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
  leftArm = "pose_left",
  rightArm = "pose_right",
  bothArms = "pose_both",

  none = "none"
}

interface PoseButtonProps {
  buttonText: string;
  disabled: boolean;
  ros: ROSLIB.Ros | null;
  setGameActionOpts: (gas: GameAction[]) => void;
  setActionType: () => void;
}

const PoseButton: React.FunctionComponent<PoseButtonProps> = ({
  buttonText,
  disabled,
  ros,
  setGameActionOpts,
  setActionType
}) => {
  return (
    <button
      type="button"
      disabled={disabled}
      onClick={(): void => {
        const searchSeqClient = new ROSLIB.Service({
          ros: ros as ROSLIB.Ros,
          name: "/search_pose",
          serviceType: "flo_core_defs/SearchPose"
        });
        console.log("connected to service to search for a pose");

        const request = new ROSLIB.ServiceRequest({ search: "" });

        searchSeqClient.callService(request, resp => {
          const poses = [];
          for (let i = 0; i < resp.ids.length; i += 1) {
            poses.push({
              id: resp.ids[i],
              description: resp.poses[i].description
            });
          }
          setGameActionOpts(poses);
          console.log("received pose ");
        });
        console.log("searched for all poses");
        setActionType();
      }}
    >
      {buttonText}
    </button>
  );
};

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
        <div>
          <button
            type="button"
            disabled={!connected || actionType == ActionType.seq}
            onClick={(): void => {
              const searchSeqClient = new ROSLIB.Service({
                ros: ros as ROSLIB.Ros,
                name: "/search_pose_seq",
                serviceType: "flo_core_defs/SearchPoseSeq"
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

          <PoseButton
            buttonText="Pose - Right Arm"
            disabled={!connected || actionType == ActionType.rightArm}
            ros={ros}
            setGameActionOpts={setGameActionOpts}
            setActionType={(): void => {
              setActionType(ActionType.rightArm);
            }}
          />

          <PoseButton
            buttonText="Pose - Left Arm"
            disabled={!connected || actionType == ActionType.leftArm}
            ros={ros}
            setGameActionOpts={setGameActionOpts}
            setActionType={(): void => {
              setActionType(ActionType.leftArm);
            }}
          />
        </div>

        <div>
          <label htmlFor="toSay">
            To Say:
            <input
              type="text"
              name="toSay"
              value={toSay}
              onChange={(e): void => setToSay(e.target.value)}
            />
          </label>
        </div>

        <div>
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
        </div>

        <div>
          {gameActionOpts.map((value, idx) => (
            <GameActionOpt
              key={actionType + value.id}
              action={value}
              setActiveOpt={(): void => {
                setActiveOpt(idx);
              }}
              active={idx == activeOpt}
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
          onClick={(): void => {
            const dt = new Date();
            addGameAction({
              type: actionType,
              text: toSay,
              id: activeOpt,
              time: timeTarget,
              desc: gameActionOpts[activeOpt].description,
              key: actionType + activeOpt + dt.getTime()
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
  const [saveID, setSaveID] = useState(0);
  return (
    <div style={basicBlock}>
      <h2>GameBuckets</h2>

      <button
        type="button"
        onClick={(): void => {
          //TODO: Implement this
          console.warn("not implemente");
        }}
      >
        Load Bucket
      </button>
      <button
        type="button"
        onClick={(): void => {
          //TODO: Implement this
          console.warn("not implemente");
        }}
      >
        Save Bucket
      </button>
      <button
        type="button"
        onClick={(): void => {
          setShowAdd(true);
        }}
      >
        Add Item
      </button>
      <hr />

      <div style={{ display: "flex", flexDirection: "column-reverse" }}>
        {steps.map((value, idx) => (
          <GameStep
            def={value}
            del={(): void => {
              const stepsT = Array.from(steps);
              stepsT.splice(idx, 1);
              setSteps(stepsT);
            }}
            key={value.key}
          />
        ))}
      </div>

      <AddGameAction
        ros={ros}
        addGameAction={(action: StepDef): void => {
          setSteps([...steps, action]);
          setShowAdd(false);
        }}
        cancel={(): void => {
          setShowAdd(false);
        }}
        showAdd={showAdd}
        connected={connected}
      />
    </div>
  );
};

export default GameBuckets;
