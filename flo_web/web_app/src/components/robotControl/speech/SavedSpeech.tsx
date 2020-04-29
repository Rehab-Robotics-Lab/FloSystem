import React, { useEffect, useState } from "react";
import * as ROSLIB from "roslib";
import { SetSpeechTarget, Utterance } from "../../robotController";
import ModalWrapper from "../ModalWrapper";

const shrinkString = (str: string): string => {
  let ret = str;
  if (str.length > 25) {
    ret = str.slice(0, 10) + "..." + str.slice(-9);
  }
  return ret;
};

interface UtteranceID extends Utterance {
  id: number;
}

interface UtteranceProps {
  utterance: UtteranceID;
  setSpeechTarget: SetSpeechTarget;
  disabled: boolean;
}

const UtteranceCont: React.FunctionComponent<UtteranceProps> = ({
  utterance,
  setSpeechTarget,
  disabled
}) => {
  return (
    <button
      type="button"
      onClick={(): void => {
        setSpeechTarget(utterance);
      }}
      disabled={disabled}
    >
      {shrinkString(utterance.text)}
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
  const [utterances, setUtterances] = useState<UtteranceID[]>([]);
  const [showSave, setShowSave] = useState(false);
  const [saveID, setSaveID] = useState(0);
  const [setUtterSrv, setSetUtterSrv] = useState<ROSLIB.Service | null>(null);

  const saveSpeech = (): void => {
    const req = new ROSLIB.ServiceRequest({
      text: speechTarget.text,
      metadata: speechTarget.metadata === null ? "" : speechTarget.metadata,
      id: saveID,
      filename: speechTarget.fileLocation
    });
    // send service call to add new utterance
    if (setUtterSrv === null) {
      return;
    }
    setUtterSrv.callService(req, res => {
      const targetId = utterances.findIndex(item => item.id === res.id);
      const utterancesT = [...utterances];
      if (targetId === -1) {
        utterancesT.push({
          id: res.id,
          text: speechTarget.text,
          metadata: speechTarget.metadata,
          fileLocation: speechTarget.fileLocation
        });
      } else {
        utterancesT[targetId] = {
          id: res.id,
          text: speechTarget.text,
          metadata: speechTarget.metadata,
          fileLocation: speechTarget.fileLocation
        };
      }
      setUtterances(utterancesT);
      setSaveID(0);
      console.log("got response from set utterance service");
    });
    console.log("sent request to set utterance service");
    //
    // read back response and add to the list
  };
  // get all of the utterances
  useEffect(() => {
    if (!connected) return;

    const searchUtterances = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/search_utterance",
      serviceType: "flo_core_defs/SearchUtterance"
    });
    console.log("connected to search utterances service");

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
      console.log("got response from search utterances service");
    });

    const setUtterSrvT = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/set_utterance",
      serviceType: "flo_core_defs/SetUtterance"
    });
    setSetUtterSrv(setUtterSrvT);
    console.log("connected to set utterances service");
  }, [connected, ros]);

  return (
    <div
      style={{
        maxWidth: "300px",
        overflow: "hidden",
        display: "flex",
        flexDirection: "column"
      }}
    >
      <h3>Saved Utterances</h3>
      <button
        type="button"
        onClick={(): void => {
          setShowSave(true);
        }}
        disabled={!speechTarget.fileLocation}
      >
        Save
      </button>
      <hr />

      <div
        style={{
          display: "flex",
          flexDirection: "column",
          overflow: "auto",
          maxHeight: "400px"
        }}
      >
        {utterances.map(value => (
          <UtteranceCont
            utterance={value}
            setSpeechTarget={setSpeechTarget}
            key={value.id}
            disabled={speaking}
          />
        ))}
      </div>

      <ModalWrapper show={showSave}>
        <h3>Save an Utterance or Overwrite an Exisiting One</h3>
        <label htmlFor="saveUtterIDSelector">
          Save As:
          <select
            id="saveUtterIDSeelector"
            onChange={(obj): void => {
              const newId: number = parseInt(obj.target.value, 10);
              setSaveID(newId);
            }}
          >
            <option value="0">New Utterance</option>
            {utterances.map((value, idx) => (
              <option key={idx} value={value.id}>
                {shrinkString(value.text)}
              </option>
            ))}
          </select>
        </label>

        <button
          type="button"
          onClick={(): void => {
            saveSpeech();
            setShowSave(false);
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
    </div>
  );
};

export default SavedSpeech;
