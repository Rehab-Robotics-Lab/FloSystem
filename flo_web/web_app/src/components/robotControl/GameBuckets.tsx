import React, { useState, useEffect } from "react";
import * as ROSLIB from "roslib";
import { basicBlock } from "../../styleDefs/styles";
import ModalWrapper from "./ModalWrapper";

interface SaveBucketProps {
  ros: ROSLIB.Ros | null;
  showSave: boolean;
  cancel: () => void;
  connected: boolean;
  steps: StepDef[];
  setBucketId: (id: number, def: GameBucket) => void;
  buckets: GameBucket[];
}

const SaveBucket: React.FunctionComponent<SaveBucketProps> = ({
  ros,
  showSave,
  cancel,
  connected,
  steps,
  setBucketId,
  buckets,
}) => {
  const [saveID, setSaveID] = useState(0);
  const [nme, setNme] = useState("");
  const [desc, setDesc] = useState("");
  const [subjNo, setSubjNo] = useState(0);
  const [targGame, setTargGame] = useState<"simon_says" | "target_touch">(
    "simon_says"
  );

  const save = (): void => {
    if (!connected) {
      return;
    }
    if (steps.length === 0) {
      return;
    }
    if (nme === "") {
      return;
    }
    const serv = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/set_game_bucket",
      serviceType: "flo_core_defs/SetGameBucket",
    });

    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    const cleanSteps = steps.map(({ desc, key, ...keep }) => keep);
    console.log(steps);
    console.log(cleanSteps);

    const thisBucket = {
      name: nme,
      subject: subjNo,
      targeted_game: targGame,
      description: desc,
      steps: steps,
    };

    const cleanBucket = {
      name: nme,
      subject: subjNo,
      targeted_game: targGame,
      description: desc,
      steps: cleanSteps,
    };

    console.log(cleanBucket);

    const req = new ROSLIB.ServiceRequest({
      id: saveID,
      game_bucket: cleanBucket,
    });
    serv.callService(
      req,
      (resp) => {
        setBucketId(resp.id, thisBucket);
        cancel();
      },
      (resp) => {
        alert(`failed to save bucket: ${resp}`);
      }
    );
  };
  return (
    <ModalWrapper show={showSave}>
      <h1>Save a New Game Bucket or Overwrite an Existing One</h1>
      <label htmlFor="saveBucketIDSelector">
        Save As:
        <select
          id="saveBucketIDSelector"
          onChange={(obj): void => {
            const newId: number = parseInt(obj.target.value, 10);
            setSaveID(newId);
            if (newId > 0) {
              setDesc(buckets[newId].description);
              setNme(buckets[newId].name);
              setSubjNo(buckets[newId].subject);
              const gt = buckets[newId].targeted_game;
              if (["simon_says", "target_touch"].includes(gt)) {
                setTargGame(gt as "simon_says" | "target_touch");
              }
            }
          }}
        >
          <option value="0">New Bucket</option>
          {buckets.map((value, idx) => (
            <option key={idx} value={idx}>
              {value.name}
            </option>
          ))}
        </select>
      </label>

      <label htmlFor="bucketNameSave">
        Name:
        <input
          id="bucktNameSave"
          type="text"
          value={nme}
          onChange={(obj): void => {
            setNme(obj.target.value);
          }}
        />
      </label>

      <label htmlFor="bucketDescSave">
        Description:
        <input
          id="bucketDescSave"
          type="text"
          value={desc}
          onChange={(obj): void => {
            setDesc(obj.target.value);
          }}
        />
      </label>

      <label htmlFor="bucketSubjSave">
        Subject Number:
        <input
          id="bucketSubjSave"
          type="number"
          min="0"
          value={subjNo}
          onChange={(obj): void => {
            setSubjNo(parseInt(obj.target.value, 10));
          }}
        />
      </label>

      <label htmlFor="bucketGameTypeSave">
        Game Type:
        <select
          id="bucketGameTypeSave"
          onChange={(obj): void => {
            const gt = obj.target.value;
            if (["simon_says", "target_touch"].includes(gt)) {
              setTargGame(gt as "simon_says" | "target_touch");
            }
          }}
        >
          <option value="simon_says">Simon Says</option>
          <option value="target_touch">Target Touch</option>
        </select>
      </label>

      <button type="button" onClick={(): void => save()} disabled={!connected}>
        Save
      </button>
      <button type="button" onClick={(): void => cancel()}>
        Cancel
      </button>
    </ModalWrapper>
  );
};

interface StepDef {
  type: string;
  text: string;
  id: number;
  time: number;
  desc: string;
  key: string;
}

interface LoadBucketProps {
  showLoad: boolean;
  cancel: () => void;
  buckets: GameBucket[];
  setSteps: (arg: StepDef[]) => void;
}

const LoadBucket: React.FunctionComponent<LoadBucketProps> = ({
  showLoad,
  cancel,
  buckets,
  setSteps,
}) => {
  const [loadID, setLoadID] = useState(1);
  const load = (): void => {
    setSteps(buckets[loadID].steps);
    cancel();
  };
  return (
    <ModalWrapper show={showLoad}>
      <h1>Load Game Bucket</h1>
      <label htmlFor="loadBucketSelector">
        <select
          id="loadBucketSelector"
          onChange={(obj): void => {
            setLoadID(parseInt(obj.target.value, 10));
          }}
        >
          {buckets.map((value, idx) => (
            <option key={idx} value={idx}>
              {value.name}
            </option>
          ))}
        </select>
      </label>

      <button
        type="button"
        onClick={(): void => load()}
        disabled={buckets.length < 2}
      >
        Load
      </button>
      <button type="button" onClick={(): void => cancel()}>
        Cancel
      </button>
    </ModalWrapper>
  );
};

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
  active,
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

  none = "none",
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
  setActionType,
}) => {
  return (
    <button
      type="button"
      disabled={disabled}
      onClick={(): void => {
        const searchSeqClient = new ROSLIB.Service({
          ros: ros as ROSLIB.Ros,
          name: "/search_pose",
          serviceType: "flo_core_defs/SearchPose",
        });
        console.log("connected to service to search for a pose");

        const request = new ROSLIB.ServiceRequest({ search: "" });

        searchSeqClient.callService(request, (resp) => {
          const poses = [];
          for (let i = 0; i < resp.ids.length; i += 1) {
            poses.push({
              id: resp.ids[i],
              description: resp.poses[i].description,
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
  connected,
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
            disabled={!connected || actionType === ActionType.seq}
            onClick={(): void => {
              const searchSeqClient = new ROSLIB.Service({
                ros: ros as ROSLIB.Ros,
                name: "/search_pose_seq",
                serviceType: "flo_core_defs/SearchPoseSeq",
              });
              console.log("connected to service to search for a pose sequence");

              const request = new ROSLIB.ServiceRequest({ search: "" });

              searchSeqClient.callService(request, (resp) => {
                const seqs = [];
                for (let i = 0; i < resp.ids.length; i += 1) {
                  seqs.push({
                    id: resp.ids[i],
                    description: resp.sequences[i].description,
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
            disabled={!connected || actionType === ActionType.rightArm}
            ros={ros}
            setGameActionOpts={setGameActionOpts}
            setActionType={(): void => {
              setActionType(ActionType.rightArm);
            }}
          />

          <PoseButton
            buttonText="Pose - Left Arm"
            disabled={!connected || actionType === ActionType.leftArm}
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
              active={idx === activeOpt}
            />
          ))}
        </div>
        <button
          type="button"
          disabled={
            actionType === ActionType.none ||
            timeTarget < 0 ||
            activeOpt < 0 ||
            activeOpt >= gameActionOpts.length
          }
          onClick={(): void => {
            const dt = new Date();
            addGameAction({
              type: actionType,
              text: toSay,
              id: gameActionOpts[activeOpt].id,
              time: timeTarget,
              desc: gameActionOpts[activeOpt].description,
              key: actionType + activeOpt + dt.getTime(),
            });
          }}
        >
          Add
        </button>
      </div>
    </ModalWrapper>
  );
};

export interface GameBucket {
  name: string;
  subject: number;
  targeted_game: string;
  description: string;
  steps: StepDef[];
}

// Takes a parameter ros, which is the connection to ros
const GameBuckets: React.FunctionComponent<GameBucketsProps> = ({
  ros,
  connected,
}) => {
  const [steps, setSteps] = useState<StepDef[]>([]);
  const [showAdd, setShowAdd] = useState(false);
  const [showSave, setShowSave] = useState(false);
  const [showLoad, setShowLoad] = useState(false);
  const [buckets, setBuckets] = useState<GameBucket[]>([]);

  useEffect(() => {
    if (!connected) {
      return;
    }
    const srv = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/search_game_bucket_name_desc",
      serviceType: "flo_core_defs/SearchGameBucket",
    });
    console.log("Connected to service to search for a game bucket");
    const request = new ROSLIB.ServiceRequest({ search: "" });
    srv.callService(request, (resp) => {
      const tmpBuckets = [];
      for (let idx = 0; idx < resp["ids"].length; idx++) {
        tmpBuckets[resp["ids"][idx]] = resp["game_buckets"][idx];
      }
      setBuckets(tmpBuckets);
    });
  }, [setBuckets, ros, connected]);

  const setBucketID = (id: number, bucket: GameBucket): void => {
    buckets[id] = bucket;
    setBuckets(buckets);
  };

  return (
    <div style={basicBlock}>
      <h2>GameBuckets</h2>

      <button
        type="button"
        onClick={(): void => {
          setShowLoad(true);
        }}
      >
        Load Bucket
      </button>
      <button
        type="button"
        onClick={(): void => {
          setShowSave(true);
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

      <div
        style={{
          display: "flex",
          flexDirection: "column-reverse",
          overflowY: "auto",
        }}
      >
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
      <SaveBucket
        ros={ros}
        showSave={showSave}
        cancel={(): void => {
          setShowSave(false);
        }}
        connected={connected}
        steps={steps}
        setBucketId={setBucketID}
        buckets={buckets}
      />
      <LoadBucket
        showLoad={showLoad}
        cancel={(): void => {
          setShowLoad(false);
        }}
        buckets={buckets}
        setSteps={(steps: StepDef[]): void => {
          setSteps(steps);
        }}
      />
    </div>
  );
};

export default GameBuckets;
