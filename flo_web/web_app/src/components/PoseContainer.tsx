import React, { useEffect, useState } from "react";
import * as ROSLIB from "roslib";

export interface PoseMsg {
  description: string;
  joint_names: string[];
  joint_positions: number[];
}

export interface PoseWrapper {
  pose: PoseMsg;
  id: number;
}

interface PoseProps {
  pose: PoseWrapper;
  addToMoveList: Function;
}

interface JointState {
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
}

interface SearchPoseResp {
  poses: PoseMsg[];
  ids: number[];
}

interface PoseObj {
  id: number;
  pose: PoseMsg;
}

const Pose: React.FunctionComponent<PoseProps> = ({ pose, addToMoveList }) => {
  return (
    <button
      type="button"
      onClick={() => {
        addToMoveList(pose);
      }}
    >
      {pose.pose.description}
    </button>
  );
};

// TODO: specific add to move list
interface PoseContainerProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  addToMoveList: Function;
}

// Takes a parameter ros, which is the connection to ros
const PoseContainer: React.FunctionComponent<PoseContainerProps> = ({
  ros,
  connected,
  addToMoveList
}) => {
  const [PosesList, setPosesList] = useState<PoseObj[]>([]);
  const [showSave, setShowSave] = useState(false);
  const [saveLR, setSaveLR] = useState<"left" | "right">("right");
  const [saveID, setSaveID] = useState(0);
  const [saveDescription, setSaveDescription] = useState("");
  const [pose, setPose] = useState<JointState | null>(null);
  const [poseListener, setPoseListener] = useState<ROSLIB.Topic | null>(null);
  const [setPoseSrv, setSetPoseSrv] = useState<ROSLIB.Service | null>(null);

  // get all of the poses
  useEffect(() => {
    if (!connected) return;

    const searchPosesClient = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/search_pose",
      serviceType: "flo_core/SearchPose"
    });

    const request = new ROSLIB.ServiceRequest({ search: "" });

    searchPosesClient.callService(request, resp => {
      const poses = [];
      for (let i = 0; i < resp.ids.length; i += 1) {
        poses.push({ id: resp.ids[i], pose: resp.poses[i] });
      }
      setPosesList(poses);
    });

    // TODO: Figure out how to clean up pose listener
    // poseListener.unsubscribe();
    // setPoseListener(null);
    const poseListenerT = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "joint_states",
      messageType: "sensor_msgs/JointState"
    });
    poseListenerT.subscribe(msg => {
      setPose(msg as JointState);
    });
    setPoseListener(poseListenerT);

    const setPoseSrvT = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/set_pose",
      serviceType: "flo_core/SetPose"
    });
    setSetPoseSrv(setPoseSrvT);
  }, [connected, ros]);

  return (
    <div
      id="poses-container"
      style={{
        maxWidth: "150px",
        backgroundColor: "white",
        borderRadius: "25px",
        padding: "10px",
        margin: "10px"
      }}
    >
      <h2>Poses:</h2>

      <button
        type="button"
        onClick={(): void => {
          setShowSave(true);
        }}
        disabled={!connected}
      >
        Save Pose
      </button>
      <hr />
      {showSave && (
        <div
          style={{
            position: "absolute",
            bottom: 0,
            left: 0,
            right: 0,
            top: 0,
            background: "rgba(0,0,0,.3)",
            display: "flex",
            justifyContent: "center",
            alignItems: "center"
          }}
        >
          <div
            style={{
              width: "200px",
              background: "white",
              borderRadius: "10px",
              minWidth: "500px",
              position: "relative",
              textAlign: "center",
              display: "flex",
              flexDirection: "column"
            }}
          >
            <h3>Save a New Pose or Overwrite an Existing One</h3>
            <label htmlFor="saveLRButton">
              Arm to save:
              <button
                id="saveLRButton"
                type="button"
                onClick={(): void => {
                  if (saveLR === "right") {
                    setSaveLR("left");
                  } else {
                    setSaveLR("right");
                  }
                }}
              >
                {saveLR}
              </button>{" "}
            </label>
            <label htmlFor="savePoseIDSelector">
              Save As:
              <select
                id="savePoseIDSelector"
                onChange={(obj): void => {
                  const newId = parseInt(obj.target.value, 10);
                  setSaveID(newId);
                  if (newId > 0) {
                    const newDesc =
                      obj.target[obj.target.selectedIndex].textContent;
                    setSaveDescription(newDesc as string); //TODO: why is this as needed?
                  }
                }}
              >
                <option value="0">New Pose</option>
                {PosesList.map(value => (
                  <option value={value.id}>{value.pose.description}</option>
                ))}
              </select>
            </label>
            <label htmlFor="savePoseDescription">
              Description:
              <input
                type="text"
                value={saveDescription}
                onChange={obj => {
                  setSaveDescription(obj.target.value);
                }}
              />
            </label>

            <button
              type="button"
              disabled={!connected || !saveDescription}
              onClick={() => {
                if (pose === null) {
                  return;
                }
                const jointsOfIterest = [];
                const cleanNames = [];
                const pos = [];
                for (let idx = 0; idx < pose.name.length; idx += 1) {
                  if (pose.name[idx].includes(saveLR)) {
                    jointsOfIterest.push(idx);
                    cleanNames.push(pose.name[idx].slice(saveLR.length + 1));
                    pos.push(pose.position[idx]);
                  }
                }
                const newPose = {
                  description: saveDescription,
                  joint_names: cleanNames,
                  joint_positions: pos
                };
                const req = new ROSLIB.ServiceRequest({
                  pose: newPose,
                  id: saveID
                });

                if (setPoseSrv === null) {
                  //TODO: throw an error
                  return;
                }
                setPoseSrv.callService(req, res => {
                  const targetId = PosesList.findIndex(
                    item => item.id === res.id
                  );
                  const PosesListT = [...PosesList];
                  if (targetId === -1) {
                    PosesListT.push({
                      id: res.id,
                      pose: newPose
                    });
                  } else {
                    PosesListT[targetId] = { id: res.id, pose: newPose };
                  }
                  setPosesList(PosesListT);
                  setShowSave(false);
                  setSaveID(0);
                  setSaveDescription("");
                });

                // ros serve save id
              }}
            >
              Save
            </button>
            <button
              type="button"
              onClick={() => {
                setShowSave(false);
              }}
            >
              Cancel
            </button>
          </div>
        </div>
      )}

      <div
        style={{
          display: "flex",
          flexDirection: "column",
          overflow: "auto",
          maxHeight: "400px"
        }}
      >
        {PosesList.map(value => (
          <Pose key={value.id} pose={value} addToMoveList={addToMoveList} />
        ))}
      </div>
    </div>
  );
};

export default PoseContainer;
