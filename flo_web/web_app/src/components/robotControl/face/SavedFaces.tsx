import React, { useEffect, useState } from "react";
import * as ROSLIB from "roslib";
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
      style={{ wordWrap: "break-word" }}
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
      setEyeOptions(res.available_eye_directions);
      console.log("received back available eye directions");
    });
    console.log("requested available eye directions");
    //
    // read back response and add to the list
  };
  // get all of the utterances
  useEffect(() => {
    if (!connected) return;

    const getFaceOptions = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/get_face_options",
      serviceType: "flo_face_defs/GetFaceOptions"
    });
    console.log("connected to service to get face options");

    const request = new ROSLIB.ServiceRequest({});

    getFaceOptions.callService(request, resp => {
      setFaces(resp.faces);
      console.log("received back available faces");
    });
    console.log("requested available faces");

    const setFaceSrvT = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/set_face",
      serviceType: "flo_face_defs/SetFace"
    });
    setSetFaceSrv(setFaceSrvT);
    console.log("connected to service to set face");
  }, [connected, ros]);

  return (
    <div
      style={{ display: "flex", flexDirection: "column", overflow: "hidden" }}
    >
      <h3>Available Faces:</h3>
      <div
        style={{
          display: "flex",
          flexDirection: "column",
          overflowY: "auto",
          width: "auto",
          maxHeight: "400px"
        }}
      >
        {faces.map(value => (
          <SavedFace
            name={value}
            setFace={() => setFace(value)}
            disabled={!connected}
            key={value}
          />
        ))}
      </div>
    </div>
  );
};

export default SavedFaces;
