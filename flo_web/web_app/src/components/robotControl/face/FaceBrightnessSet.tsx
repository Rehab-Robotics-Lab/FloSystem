import React, { useEffect, useState } from "react";
import * as ROSLIB from "roslib";
import { FaceState } from "./FaceContainer";

interface FaceBrightnessSetProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  faceState: null | FaceState;
}

// Takes a parameter ros, which is the connection to ros
const FaceBrightnessSet: React.FunctionComponent<FaceBrightnessSetProps> = ({
  ros,
  connected,
  faceState
}) => {
  const [
    setBrightnessSrv,
    setSetBrightnessSrv
  ] = useState<ROSLIB.Service | null>(null);

  const setBrightness = (value: number): void => {
    const req = new ROSLIB.ServiceRequest({
      target: "all",
      value: value
    });
    // send service call to add new utterance
    if (setBrightnessSrv === null) {
      return;
    }
    setBrightnessSrv.callService(req, function() {
      //do nothing
      console.log("succesfully changed eye brightness");
    });
    console.log("set eye brightness");
  };

  useEffect(() => {
    if (!connected) return;
    const setBrightnessSrvT = new ROSLIB.Service({
      ros: ros as ROSLIB.Ros,
      name: "/set_face_brightness",
      serviceType: "flo_face_defs/SetFaceBrightness"
    });
    setSetBrightnessSrv(setBrightnessSrvT);
    console.log("connected to set eye brightness service");
  }, [connected, ros]);

  const brightness = faceState ? faceState.mouth_brightness : 0;
  return (
    <div>
      <h3>Brightness:</h3>
      <input
        type="range"
        min="0"
        max="15"
        value={brightness}
        onChange={(e): void => setBrightness(parseInt(e.target.value))}
      />
    </div>
  );
};

export default FaceBrightnessSet;
