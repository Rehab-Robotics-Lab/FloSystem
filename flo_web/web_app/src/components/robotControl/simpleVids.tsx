import React, { useEffect, useState, CSSProperties, Ref } from "react";
import * as ROSLIB from "roslib";
import { wrapStyle } from "../../styleDefs/styles";
import { useParams } from "react-router-dom";
//import adapter from "webrtc-adapter";
import { WebrtcRos } from "../../externalLibs/webrtc_ros.js";
import axios from "axios";
import SystemMonitor from "./SystemMonitor";

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
  const remoteRefUpper = React.useRef(null);
  const localRef = React.useRef(null);
  const upperStream = React.useRef<MediaStream>();
  const localStream = React.useRef<MediaStream>();

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

      let connection1: any;

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
          connection1 = WebrtcRos.createConnection(
            connectionString,
            serverConfig
          );
          console.log("connected to webrtc ros");

          connection1.onConfigurationNeeded = (): void => {
            const remoteStreamConfigUpper = { video: {}, audio: {} };
            remoteStreamConfigUpper.video = {
              id: "subscribed_video_upper",
              src: "ros_image:/upper_realsense/color/image_raw"
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

          console.log("*** Done starting webrtc ***");
        });
      return (): void => {
        console.log("*** Close webrtc connections ***");
        connection1.close();
        const closeStreams = (stream: MediaStream): void => {
          if (stream) {
            stream.getTracks().forEach((track: any) => {
              track.stop();
            });
          }
        };
        localStream && localStream.current && closeStreams(localStream.current);
      };
    }
  }, [robotName, connected, ipAddr]);
  //<script type="text/javascript" src={"/web/adapter.js"} />
  //

  const vidStyle: CSSProperties = {
    width: "100%",
    height: "auto"
  };

  const vidContainerStyle: CSSProperties = {
    width: "40%",
    minWidth: "150px",
    display: "flex",
    flexDirection: "column",
    height: "auto",
    borderRadius: "5px",
    overflow: "hidden"
  };

  return (
    <>
      <div style={vidContainerStyle}>
        <video
          ref={remoteRefUpper}
          id="remote-video-upper"
          autoPlay={true}
          style={vidStyle}
        ></video>
      </div>
      <div
        style={Object.assign({}, vidContainerStyle, {
          minWidth: "55px",
          width: "30%"
        })}
      >
        <video
          ref={localRef}
          id="local-video"
          autoPlay={true}
          style={vidStyle}
          muted={true}
        ></video>
        <div style={{ display: "flex", flexWrap: "wrap" }}>
          <SystemMonitor ros={ros} connected={connected} />
        </div>
      </div>
    </>
  );
};

export default Vids;
