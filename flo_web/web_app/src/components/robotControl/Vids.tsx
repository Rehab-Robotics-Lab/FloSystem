import React, { useEffect, useState, CSSProperties } from "react";
import * as ROSLIB from "roslib";
import { wrapStyle } from "../../styleDefs/styles";
import { useParams } from "react-router-dom";
//import adapter from "webrtc-adapter";
import { WebrtcRos } from "../../externalLibs/webrtc_ros.js";
import axios from "axios";

interface VidsProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  ipAddr: string;
}

interface TurnCreds {
  username: string;
  password: string;
}

interface IceDef {
  urls: string | string[];
  credential?: string;
  username?: string;
}
interface ServerConfig {
  iceServers: IceDef[];
  iceCandidatePoolSize: number;
}

// Takes a parameter ros, which is the connection to ros
const Vids: React.FunctionComponent<VidsProps> = ({
  ros,
  connected,
  ipAddr
}) => {
  const remoteRef = React.useRef(null);
  const localRef = React.useRef(null);
  const [localEnable, setLocalEnable] = useState(true);
  const [remoteEnable, setRemoteEnable] = useState(true);
  const remoteStream = React.useRef(null);
  const localStream = React.useRef(null);

  const { robotName } = useParams();

  useEffect(() => {
    if (connected) {
      const getTurnCreds = async (): Promise<TurnCreds> => {
        const resp = await axios.get(
          `/api/webrtc/turn-credentials?robotName=${robotName}`
        );
        return resp.data as TurnCreds;
      };

      const connectionString =
        "wss://" + ipAddr + "/robot/" + robotName + "/webrtc";
      const serverConfig: ServerConfig = {
        iceServers: [
          {
            urls: [
              "stun:stun1.l.google.com:19302",
              "stun:stun2.l.google.com:19302"
            ]
          }
        ],
        iceCandidatePoolSize: 10
      };
      getTurnCreds()
        .then(turnCredentials => {
          serverConfig.iceServers.push(
            {
              urls: `turn:${ipAddr}:5349?transport=udp`,
              username: turnCredentials["username"],
              credential: turnCredentials["password"]
            },
            {
              urls: `turn:${ipAddr}:3478?transport=udp`,
              username: turnCredentials["username"],
              credential: turnCredentials["password"]
            },
            {
              urls: `turn:${ipAddr}:5349?transport=tcp`,
              username: turnCredentials["username"],
              credential: turnCredentials["password"]
            },
            {
              urls: `turn:${ipAddr}:3478?transport=tcp`,
              username: turnCredentials["username"],
              credential: turnCredentials["password"]
            }
          );
        })
        .catch(() => {
          console.log("failed to get turn server credentials");
        })
        .finally(() => {
          const connection1 = WebrtcRos.createConnection(
            connectionString,
            serverConfig
          );
          console.log("connected to webrtc ros");

          connection1.onConfigurationNeeded = (): void => {
            const remoteStreamConfig = { video: {}, audio: {} };
            remoteStreamConfig.video = {
              id: "subscribed_video",
              //src: "ros_image:/video_to_web"
              src: "ros_image:/video_to_web"
            };
            remoteStreamConfig.audio = {
              id: "subscribed_audio",
              src: "local:"
            };

            connection1
              .addRemoteStream(remoteStreamConfig)
              .then((event: any) => {
                //stream started
                const remoteVideoElement = remoteRef as any;
                remoteVideoElement.current.srcObject = event.stream;
                event.remove.then(function() {
                  //Remote stream removed
                  remoteVideoElement.srcObject = null;
                });
                remoteStream.current = event.stream;
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

          console.log("*** Done starting webrtc ***");

          return (): void => {
            console.log("*** Close webrtc connections ***");
            connection1.close();
          };
        });
    }
  }, [robotName, connected, ipAddr]);
  //<script type="text/javascript" src={"/web/adapter.js"} />
  //

  const vidStyle = {
    width: "auto",
    maxWidth: "300px"
  };

  return (
    <>
      <div style={wrapStyle}>
        <video
          ref={remoteRef}
          id="remote-video"
          autoPlay={true}
          style={{ width: "auto", maxWidth: "80%" }}
        ></video>
      </div>
      <div style={wrapStyle}>
        <video
          ref={localRef}
          id="local-video"
          autoPlay={true}
          style={{ width: "auto", maxHeight: "100px" }}
          muted={true}
        ></video>
      </div>
    </>
  );
};

export default Vids;
