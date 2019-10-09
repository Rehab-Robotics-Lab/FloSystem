import React from "react";
import { basicBlock } from "../styleDefs/styles";
import * as ROSLIB from "roslib";
import { Helmet } from "react-helmet";
//import ROSRTC = require("imports-loader?window=>{}!exports-loader?window.XModule!/opt/ros/kinetic/share/webrtc_ros/web/webrtc_ros");

interface VidsProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  ipAddr: string;
  ipPort: string;
}

// Takes a parameter ros, which is the connection to ros
const Vids: React.FunctionComponent<VidsProps> = ({
  ros,
  connected,
  ipAddr,
  ipPort
}) => {
  return (
    <div style={basicBlock}>
      <Helmet>
        <script
          type="text/javascript"
          src={
            "http://" + ipAddr + ":" + (parseInt(ipPort) + 1) + "/adapter.js"
          }
        ></script>
      </Helmet>
      <video id="remote-video" autoPlay></video>
      <video id="local-video" autoPlay muted></video>
    </div>
  );
};

export default Vids;
