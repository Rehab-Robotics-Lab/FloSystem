import React, { useEffect, useState } from "react";
import * as ROSLIB from "roslib";
import { SetSpeechTarget, Utterance } from "../App";

interface UtteranceID extends Utterance {
  id: number;
}

interface UtteranceProps {
  utterance: UtteranceID;
  setSpeechTarget: SetSpeechTarget;
}

const Utterace: React.FunctionComponent<UtteranceProps> = ({
  utterance,
  setSpeechTarget
}) => {
  return (
    <button
      type="button"
      onClick={(): void => {
        setSpeechTarget(utterance);
      }}
    >
      {utterance.text}
    </button>
  );
};

interface SavedSpeechProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  speechTarget: Utterance;
  setSpeechTarget: SetSpeechTarget;
  speaking: boolean;
}
// Takes a parameter ros, which is the connection to ros
const SavedSpeech: React.FunctionComponent<SavedSpeechProps> = ({
  ros,
  connected,
  speechTarget,
  setSpeechTarget,
  speaking
}) => {
  const saveSpeechString = (str: string): void => {};
  const [utterances, setUtterances] = useState<UtteranceID[]>([]);
  const [showSave, setShowSave] = useState(false);
  const [saveID, setSaveID] = useState(0);
  const [setSeqSrv, setSetSeqSrv] = useState<ROSLIB.Service | null>(null);

  // get all of the utterances
  useEffect(() => {
    if (!connected) return;

    const searchUtterances = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/search_utterance",
      serviceType: "flo_core/SearchUtterance"
    });

    const request = new ROSLIB.ServiceRequest({ search: "" });

    searchUtterances.callService(request, resp => {
      const utterances = [];
      for (let i = 0; i < resp.ids.length; i += 1) {
        utterances.push({
          id: resp.ids[i],
          text: resp.texts[i],
          metadata: resp.metadatas[i],
          fileLocation: null
        });
      }
      setUtterances(utterances);
    });

    //const setSeqSrvT = new ROSLIB.Service({
    //ros: ros as ROSLIB.Ros,
    //name: "/set_pose_seq",
    //serviceType: "flo_core/SetPoseSeq"
    //});
    //setSetSeqSrv(setSeqSrvT);
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
      <h2>Saved Utterances</h2>
      <button
        type="button"
        onClick={(): void => {
          //saveSpeechString();
        }}
        disabled={!speechTarget.fileLocation}
      >
        Save
      </button>
      <hr />
    </div>
  );
};

export default SavedSpeech;
