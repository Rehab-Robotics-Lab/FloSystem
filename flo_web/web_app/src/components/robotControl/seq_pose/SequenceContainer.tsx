import React, { useEffect, useState } from "react";
import * as ROSLIB from "roslib";
import { SetMovesList, genRandID } from "../../robotController";
import { Move } from "./SequenceRunContainer";
import ModalWrapper from "../ModalWrapper";
import { basicBlock } from "../../../styleDefs/styles";

interface Sequence {
  pose_ids: number[];
  times: number[];
  arms: string[];
  description: string;
  total_time: number;
}

interface SequenceListItem {
  id: number;
  seq: Sequence;
}

interface SequenceProps {
  setMovesList: SetMovesList;
  getPoseSrv: ROSLIB.Service;
  sequence: SequenceListItem;
}

const Sequence: React.FunctionComponent<SequenceProps> = ({
  sequence,
  setMovesList,
  getPoseSrv
}) => {
  return (
    <button
      type="button"
      onClick={(): void => {
        const movesListT: Move[] = [];
        const seqLength = sequence.seq.arms.length;
        for (let idx = 0; idx < seqLength; idx += 1) {
          const poseId = sequence.seq.pose_ids[idx];
          const req = new ROSLIB.ServiceRequest({
            id: poseId
          });
          getPoseSrv.callService(req, res => {
            movesListT[idx] = {
              lr: sequence.seq.arms[idx] as "left" | "right",
              pose: { id: poseId, pose: res.pose },
              status: "not-run",
              time: sequence.seq.times[idx],
              key: genRandID()
            };
            setMovesList(movesListT);
          });
        }
      }}
    >
      {sequence.seq.description}
    </button>
  );
};

interface SequenceContainerProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  setMovesList: SetMovesList;
  MovesList: Move[];
}

// Takes a parameter ros, which is the connection to ros
const SequenceContainer: React.FunctionComponent<SequenceContainerProps> = ({
  ros,
  connected,
  setMovesList,
  MovesList
}) => {
  const [SeqList, setSeqList] = useState<SequenceListItem[]>([]);
  const [showSave, setShowSave] = useState(false);
  const [saveID, setSaveID] = useState(0);
  const [saveDescription, setSaveDescription] = useState("");
  const [setSeqSrv, setSetSeqSrv] = useState<ROSLIB.Service | null>(null);
  const [getPoseSrv, setGetPoseSrv] = useState<ROSLIB.Service | null>(null);

  // get all of the poses
  useEffect(() => {
    if (!connected) return;

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
          seq: resp.sequences[i]
        });
      }
      setSeqList(seqs);
      console.log("received pose sequences");
    });
    console.log("searched for all pose sequences");

    const setSeqSrvT = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/set_pose_seq",
      serviceType: "flo_core_defs/SetPoseSeq"
    });
    setSetSeqSrv(setSeqSrvT);
    console.log("connected to service to set pose sequences");

    const getPoseSrvT = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/get_pose_id",
      serviceType: "flo_core_defs/GetPoseID"
    });
    setGetPoseSrv(getPoseSrvT);
    console.log("connected to service to get poses by id");
  }, [connected, ros]);

  return (
    <div
      id="sequences-container"
      style={Object.assign({}, basicBlock, {
        maxWidth: "150px"
      })}
    >
      <h2>Sequences:</h2>
      <button
        type="button"
        onClick={(): void => {
          setShowSave(true);
        }}
        disabled={!connected}
      >
        Save Sequence
      </button>
      <hr />
      <ModalWrapper show={showSave}>
        <h3>Save a New Sequence or Overwrite an Existing One</h3>
        <label htmlFor="saveSeqIDSelector">
          Save As:
          <select
            id="saveSeqIDSelector"
            onChange={(obj): void => {
              const newId: number = parseInt(obj.target.value, 10);
              setSaveID(newId);
              if (newId > 0) {
                const newDescT: string | null =
                  obj.target[obj.target.selectedIndex].textContent;
                let newDesc = "";
                if (newDescT !== null) {
                  newDesc = newDescT;
                }

                setSaveDescription(newDesc);
              }
            }}
          >
            <option value="0">New Sequence</option>
            {SeqList.map((value, idx) => (
              <option key={idx} value={value.id}>
                {value.seq.description}
              </option>
            ))}
          </select>
        </label>
        <label htmlFor="saveSeqDescription">
          Description:
          <input
            type="text"
            value={saveDescription}
            onChange={(obj): void => {
              setSaveDescription(obj.target.value);
            }}
          />
        </label>

        <button
          type="button"
          disabled={!connected || !saveDescription}
          onClick={(): void => {
            const poseIds = [];
            const times = [];
            const arms = [];
            let totalTime = 0;
            for (let idx = 0; idx < MovesList.length; idx += 1) {
              poseIds.push(MovesList[idx].pose.id);
              times.push(MovesList[idx].time);
              arms.push(MovesList[idx].lr);
              totalTime += MovesList[idx].time;
            }

            const newSeq: Sequence = {
              pose_ids: poseIds, // eslint-disable-line
              times,
              arms,
              description: saveDescription,
              total_time: totalTime // eslint-disable-line
            };
            const req = new ROSLIB.ServiceRequest({
              sequence: newSeq,
              id: saveID
            });

            if (setSeqSrv === null) {
              return;
            }
            setSeqSrv.callService(req, res => {
              const targetId = SeqList.findIndex(item => item.id === res.id);
              const SeqListT = [...SeqList];
              if (targetId === -1) {
                SeqListT.push({
                  id: res.id,
                  seq: newSeq
                });
              } else {
                SeqListT[targetId] = {
                  id: res.id,
                  seq: newSeq
                };
              }
              setSeqList(SeqListT);
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
          onClick={(): void => {
            setShowSave(false);
          }}
        >
          Cancel
        </button>
      </ModalWrapper>
      <div
        style={{
          display: "flex",
          flexDirection: "column",
          overflow: "auto",
          maxHeight: "400px"
        }}
      >
        {getPoseSrv !== null &&
          SeqList.map(value => (
            <Sequence
              key={value.id}
              sequence={value}
              setMovesList={setMovesList}
              getPoseSrv={getPoseSrv}
            />
          ))}
      </div>
    </div>
  );
};

export default SequenceContainer;
