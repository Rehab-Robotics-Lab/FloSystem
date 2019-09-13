import React, { useEffect, useState } from "react";
import * as ROSLIB from "roslib";
import { SetSpeechTarget, Utterance } from "../App";

interface UtteranceProps extends Utterance {
  id: number;
  setSpeechTarget: SetSpeechTarget;
}

const Utterace: React.FunctionComponent<UtteranceProps> = ({
  utterance,
  setSpeechTarget
}) => {
  return (
    <button type="button" onClick={(): void => {}}>
      {utterance.text}
    </button>
  );
};

interface SavedSpeechProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  speechTarget: Utterance;
  setSpeechString: SetSpeechTarget;
  speaking: boolean;
}
// Takes a parameter ros, which is the connection to ros
const SavedSpeech: React.FunctionComponent<SavedSpeechProps> = ({
  ros,
  connected,
  speechTarget,
  setSpeechString,
  speaking
}) => {
  const saveSpeechString = (str: string): void => {};
  const [utterances, setUtterances] = useState<UtteranceProps[]>([]);
  const [showSave, setShowSave] = useState(false);
  const [saveID, setSaveID] = useState(0);
  const [setSeqSrv, setSetSeqSrv] = useState<ROSLIB.Service | null>(null);

  // get all of the utterances
  useEffect(() => {
    if (!connected) return;

    const searchUtterances = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/search_utterances",
      serviceType: "flo_core/SearchUtterances"
    });

    const request = new ROSLIB.ServiceRequest({ search: "" });

    searchUtterances.callService(request, resp => {
      const utterances = [];
      for (let i = 0; i < resp.ids.length; i += 1) {
        utterances.push({
          id: resp.ids[i],
          seq: resp.text[i]
          //WORKING HERE
        });
      }
      setSeqList(seqs);
    });

    const setSeqSrvT = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/set_pose_seq",
      serviceType: "flo_core/SetPoseSeq"
    });
    setSetSeqSrv(setSeqSrvT);

    const getPoseSrvT = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/get_pose_id",
      serviceType: "flo_core/GetPoseID"
    });
    setGetPoseSrv(getPoseSrvT);
  }, [connected, ros]);

  return (
    <div
      style={{
        maxWidth: "300px",
        backgroundColor: "white",
        borderRadius: "25px",
        padding: "10px",
        margin: "10px"
      }}
    >
      <button
        type="button"
        onClick={(): void => {
          //saveSpeechString();
        }}
        disabled={speechTarget.fileLocation === ""}
      >
        Save
      </button>
      <hr />
    </div>
  );
};

export default SavedSpeech;
