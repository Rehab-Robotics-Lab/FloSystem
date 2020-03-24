import React, { useEffect, useState } from "react";
import * as ROSLIB from "roslib";

//Populate these with getfaceoptions.srv

interface EyeOptionProps {
  name: string;
  disabled: boolean;
  setEyeDirection: () => void;
}

const EyeOption: React.FunctionComponent<EyeOptionProps> = ({
  name,
  disabled,
  setEyeDirection
}) => {
  return (
    <button
      type="button"
      disabled={disabled}
      onClick={(): void => {
        setEyeDirection();
      }}
    >
      {name}
    </button>
  );
};

interface EyeOptionsContainerProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  eyeOptions: string[];
}

// Takes a parameter ros, which is the connection to ros
const EyeOptionsContainer: React.FunctionComponent<EyeOptionsContainerProps> = ({
  ros,
  connected,
  eyeOptions
}) => {
  const [
    setEyeDirectionSrv,
    setSeteEyeDirectionSrv
  ] = useState<ROSLIB.Service | null>(null);

  const setEye = (name: string): void => {
    const req = new ROSLIB.ServiceRequest({
      direction: name
    });
    // send service call to add new utterance
    if (setEyeDirectionSrv === null) {
      return;
    }
    setEyeDirectionSrv.callService(req, function() {
      //do nothing.
    });
    console.log("set eye direction");
  };

  useEffect(() => {
    if (!connected) return;
    const setEyeDirectionSrvT = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/set_eye_direction",
      serviceType: "flo_face_defs/SetEyeDirection"
    });
    setSeteEyeDirectionSrv(setEyeDirectionSrvT);
    console.log("connected to set eye direction service");
  }, [connected, ros]);

  return (
    <div>
      <h3>Available Eye Options:</h3>
      <div
        style={{
          display: "flex",
          flexDirection: "column",
          overflow: "auto",
          maxHeight: "400px"
        }}
      >
        {eyeOptions.map(value => (
          <EyeOption
            name={value}
            disabled={!connected}
            setEyeDirection={(): void => setEye(value)}
            key={value}
          />
        ))}
      </div>
    </div>
  );
};

export default EyeOptionsContainer;
