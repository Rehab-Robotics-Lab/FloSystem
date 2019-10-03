import React, { useEffect, useState } from "react";
import * as ROSLIB from "roslib";
import { SetSpeechTarget, Utterance } from "../App";
import { basicBlock } from "../styleDefs/styles";
import { SetEyeOptions } from "./FaceContainer";

//Populate these with getfaceoptions.srv
interface Face {
  name: string;
}

interface SetFace {
  (): void;
}

interface SavedFaceProps extends Face {
  setFace: SetFace;
  disabled: boolean;
}

const SavedFace: React.FunctionComponent<SavedFaceProps> = ({
  name,
  setFace,
  disabled
}) => {
  return (
    <button
      type="button"
      disabled={disabled}
      onClick={(): void => {
        setFace();
      }}
    >
      {name}
    </button>
  );
};

interface SavedFacesProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  setEyeOptions: SetEyeOptions;
}

// Takes a parameter ros, which is the connection to ros
const SavedFaces: React.FunctionComponent<SavedFacesProps> = ({
  ros,
  connected,
  setEyeOptions
}) => {
  const [setFaceSrv, setSetFaceSrv] = useState<ROSLIB.Service | null>(null);
  const [faces, setFaces] = useState<string[]>([]);

  const setFace = (name: string): void => {
    const req = new ROSLIB.ServiceRequest({
      face: name
    });
    // send service call to add new utterance
    if (setFaceSrv === null) {
      return;
    }
    setFaceSrv.callService(req, res => {
      setEyeOptions(res);
    });
    //
    // read back response and add to the list
  };
  // get all of the utterances
  useEffect(() => {
    if (!connected) return;

    const getFaceOptions = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/get_face_options",
      serviceType: "flo_face/GetFaceOptions"
    });

    const request = new ROSLIB.ServiceRequest({});

    getFaceOptions.callService(request, resp => {
      setFaces(resp.faces);
    });

    const setFaceSrvT = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/set_face",
      serviceType: "flo_face/SetFace"
    });
    setSetFaceSrv(setFaceSrvT);
  }, [connected, ros]);

  return (
    <div>
      <h3>Available Faces:</h3>
      <div
        style={{
          display: "flex",
          flexDirection: "column",
          overflow: "auto",
          maxHeight: "400px"
        }}
      >
        {faces.map(value => (
          <SavedFace
            name={value}
            setFace={() => setFace(value)}
            disabled={!connected}
          />
        ))}
      </div>
    </div>
  );
};

export default SavedFaces;
