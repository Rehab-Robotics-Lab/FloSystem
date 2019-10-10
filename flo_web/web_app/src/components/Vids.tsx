import React, { useEffect, useRef } from "react";
import { basicBlock } from "../styleDefs/styles";
import * as ROSLIB from "roslib";
import { Helmet } from "react-helmet";
import adapter from "webrtc-adapter";

declare var WebrtcRos: any;

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
  const remoteRef = React.useRef(null);

  useEffect(() => {
    if (connected) {
      const connection = WebrtcRos.createConnection(
        (window.location.protocol === "https:" ? "wss://" : "ws://") +
          ipAddr +
          ":" +
          (parseInt(ipPort) + 1) +
          "/webrtc"
      );
      connection.onConfigurationNeeded = () => {
        const remote_stream_config = { video: {}, audio: {} };
        remote_stream_config.video = {
          id: "subscribed_video",
          src: "ros_image:/camera/color/image_raw"
        };
        connection.addRemoteStream(remote_stream_config).then((event: any) => {
          //stream started
          let remoteVideoElement = remoteRef as any;
          remoteVideoElement.current.srcObject = event.stream;
          event.remove.then(function(event: any) {
            //Remote stream removed
            remoteVideoElement.srcObject = null;
          });
          //(window as any).remotestream = event.stream;
        });
        connection.sendConfigure();
      };
      connection.connect();
    }
  }, [connected, ros]);
  //<script type="text/javascript" src={"/web/adapter.js"} />
  return (
    <div style={basicBlock}>
      <Helmet>
        <script type="text/javascript" src={"/web/webrtc_ros.js"} />
      </Helmet>
      <video ref={remoteRef} id="remote-video" autoPlay={true}></video>
      <video id="local-video" autoPlay={true} muted={true}></video>
    </div>
  );
};

export default Vids;
