import React, { useEffect, useState, CSSProperties } from "react";
import * as ROSLIB from "roslib";
import { wrapStyle } from "../../styleDefs/styles";
import { useParams } from "react-router-dom";
//import adapter from "webrtc-adapter";
import { Helmet } from "react-helmet";

declare var WebrtcRos: any;

interface VidsProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  ipAddr: string;
}

// Takes a parameter ros, which is the connection to ros
const Vids: React.FunctionComponent<VidsProps> = ({
  ros,
  connected,
  ipAddr
}) => {
  const remoteRefUpper = React.useRef(null);
  const remoteRefLower = React.useRef(null);
  const remoteRefFish = React.useRef(null);
  const localRef = React.useRef(null);
  const [localEnable, setLocalEnable] = useState(true);
  const [upperEnable, setUpperEnable] = useState(true);
  const [lowerEnable, setLowerEnable] = useState(true);
  const [fishEnable, setFishEnable] = useState(true);
  const upperStream = React.useRef(null);
  const lowerStream = React.useRef(null);
  const localStream = React.useRef(null);
  const fishStream = React.useRef(null);

  const { robotName } = useParams();

  useEffect(() => {
    const connectionString =
      "wss://" + ipAddr + "/robot/" + robotName + "/webrtc";
    if (connected) {
      const connection1 = WebrtcRos.createConnection(connectionString);
      console.log("connected to webrtc ros");

      connection1.onConfigurationNeeded = (): void => {
        const remoteStreamConfigUpper = { video: {}, audio: {} };
        remoteStreamConfigUpper.video = {
          id: "subscribed_video_upper",
          src: "ros_image:/upper_realsense/color/image_web"
        };
        remoteStreamConfigUpper.audio = {
          id: "subscribed_audio",
          src: "local:"
        };

        connection1
          .addRemoteStream(remoteStreamConfigUpper)
          .then((event: any) => {
            //stream started
            const remoteVideoElement = remoteRefUpper as any;
            remoteVideoElement.current.srcObject = event.stream;
            event.remove.then(function() {
              //Remote stream removed
              remoteVideoElement.srcObject = null;
            });
            upperStream.current = event.stream;
            //(window as any).remotestream = event.stream;
          });

        const userMediaConfig = { video: {}, audio: {} };
        const localStreamConfig = { video: {}, audio: {} };
        userMediaConfig.video = { width: 800, height: 480, frameRate: 20 };
        localStreamConfig.video = {
          dest: "ros_image:remote_video",
          width: 848,
          height: 480,
          frameRate: 20
        };
        userMediaConfig.audio = true;

        connection1
          .addLocalStream(userMediaConfig, localStreamConfig)
          .then(function(event: any) {
            console.log(
              "Local stream added",
              event,
              event.stream.getVideoTracks(),
              event.stream.getAudioTracks()
            );
            const localVideoElement = localRef as any;
            localVideoElement.current.srcObject = event.stream;
            event.remove.then(function() {
              //console.log("Local stream removed", event);
              localVideoElement.current.srcObject = null;
            });
            localStream.current = event.stream;
            //window.localstream = event.stream;
          });

        connection1.sendConfigure();
      };
      connection1.connect();

      const connection2 = WebrtcRos.createConnection(connectionString);

      connection2.onConfigurationNeeded = (): void => {
        const remoteStreamConfigLower = { video: {}, audio: {} };
        remoteStreamConfigLower.video = {
          id: "subscribed_video_lower",
          src: "ros_image:/lower_realsense/color/image_web"
        };

        connection2
          .addRemoteStream(remoteStreamConfigLower)
          .then((event: any) => {
            //stream started
            const remoteVideoElement = remoteRefLower as any;
            remoteVideoElement.current.srcObject = event.stream;
            event.remove.then(function() {
              //Remote stream removed
              remoteVideoElement.srcObject = null;
            });
            //(window as any).remotestream = event.stream;
            lowerStream.current = event.stream;
          });
        connection2.sendConfigure();
      };
      connection2.connect();

      const connection3 = WebrtcRos.createConnection(connectionString);

      connection3.onConfigurationNeeded = (): void => {
        const remoteStreamConfigFish = { video: {}, audio: {} };
        remoteStreamConfigFish.video = {
          id: "subscribed_video_fish",
          src: "ros_image:/fisheye_cam/image_web"
        };

        connection3
          .addRemoteStream(remoteStreamConfigFish)
          .then((event: any) => {
            //stream started
            const remoteVideoElement = remoteRefFish as any;
            remoteVideoElement.current.srcObject = event.stream;
            event.remove.then(function() {
              //Remote stream removed
              remoteVideoElement.srcObject = null;
            });
            //(window as any).remotestream = event.stream;
            fishStream.current = event.stream;
          });
        connection3.sendConfigure();
      };
      connection3.connect();

      return (): void => {
      connection1.close();
      connection2.close();
      connection3.close();
      };
    }
  }, [connected, ros, ipAddr]);
  //<script type="text/javascript" src={"/web/adapter.js"} />
  //

  const vidStyle = {
    width: "auto",
    maxWidth: "300px"
  };

  return (
    <>
      <Helmet>
      <script type="text/javascript" src={"/web/webrtc_ros.js"} />
      </Helmet>
      <div style={wrapStyle}>
        <button
          type="button"
          onClick={(): void => {
            if (upperStream && upperStream.current) {
              (upperStream!.current! as any)
                .getTracks()
                .forEach((track: any) => (track.enabled = !upperEnable));
              setUpperEnable(!upperEnable);
            }
          }}
        >
          {upperEnable ? "Disable Upper" : "Enable Upper"}
        </button>
        <video
          ref={remoteRefUpper}
          id="remote-video-upper"
          autoPlay={true}
          style={vidStyle}
        ></video>
      </div>
      <div style={wrapStyle}>
        <button
          type="button"
          onClick={(): void => {
            if (lowerStream && lowerStream.current) {
              (lowerStream!.current! as any)
                .getTracks()
                .forEach((track: any) => (track.enabled = !lowerEnable));
              setLowerEnable(!lowerEnable);
            }
          }}
        >
          {lowerEnable ? "Disable Lower" : "Enable Lower"}
        </button>
        <video
          ref={remoteRefLower}
          id="remote-video-lower"
          autoPlay={true}
          style={vidStyle}
        ></video>
      </div>
      <div style={wrapStyle}>
        <button
          type="button"
          onClick={(): void => {
            if (fishStream && fishStream.current) {
              (fishStream!.current! as any)
                .getTracks()
                .forEach((track: any) => (track.enabled = !fishEnable));
              setFishEnable(!fishEnable);
            }
          }}
        >
          {fishEnable ? "Disable Fisheye" : "Enable Fisheye"}
        </button>
        <video
          ref={remoteRefFish}
          id="remote-video-fish"
          autoPlay={true}
          style={vidStyle}
        ></video>
      </div>
      <div style={wrapStyle}>
        <button
          type="button"
          onClick={(): void => {
            if (localStream && localStream.current) {
              (localStream!.current! as any)
                .getTracks()
                .forEach((track: any) => (track.enabled = !localEnable));
              setLocalEnable(!localEnable);
            }
          }}
        >
          {localEnable ? "Disable Local" : "Enable Local"}
        </button>
        <video
          ref={localRef}
          id="local-video"
          autoPlay={true}
          style={vidStyle}
          muted={true}
        ></video>
      </div>
    </>
  );
};

export default Vids;
