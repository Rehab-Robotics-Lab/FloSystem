import React, { useState, useEffect } from "react";
import "../App.css";
import { CookiesProvider, useCookies } from "react-cookie";
import Header from "./robotControl/Header";
import URDF from "./robotControl/urdf";
import PoseContainer from "./robotControl/seq_pose/PoseContainer";
import ErrorDisplay, { ErrorItem } from "./robotControl/ErrorDisplay";
import SequenceRunContainer, {
  Move
} from "./robotControl/seq_pose/SequenceRunContainer";
import SequenceContainer from "./robotControl/seq_pose/SequenceContainer";
import colors from "../styleDefs/colors";
import SpeechContainer from "./robotControl/speech/SpeechContainer";
import MoveToPose from "./robotControl/seq_pose/MoveToPose";
import FaceContainer from "./robotControl/face/FaceContainer";
import RelaxMotors from "./robotControl/RelaxMotors";
import Vids from "./robotControl/Vids";
import * as ROSLIB from "roslib";
import Drive from "./robotControl/Drive";
import { basicBlock } from "../styleDefs/styles";
import GameContainer from "./robotControl/GameContainer";
import SystemMonitor from "./robotControl/SystemMonitor";
import GameBuckets from "./robotControl/GameBuckets";

export function genRandID(): number {
  return Math.round(Math.random() * 10000) + Date.now();
}

export interface AddError {
  (text: string, src: string): void;
}

export interface SetMovesList {
  (arg: Move[]): any;
}

export interface AddToMoveList {
  (value: Move): void;
}

export interface SetConnected {
  (con: boolean): void;
}

export interface SetMoving {
  (arg: boolean): void;
}

export interface Utterance {
  text: string;
  metadata: string | null;
  fileLocation: string | null;
}

export interface SetSpeechTarget {
  (arg: Utterance): void;
}

export interface Speech {
  id: number;
  text: string;
}

export interface SetSpeechList {
  (val: Speech): void;
}

export interface SetSpeaking {
  (val: boolean): void;
}

export interface JointState {
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
}

const RobotController: React.FunctionComponent = () => {
  const [cookies, setCookie] = useCookies(["movesList", "ipAddr", "ipPort"]);
  const [ipAddr, setIpAddr] = useState(
    cookies.ipAddr || window.location.hostname
  );
  const [ipPort, setIpPort] = useState(cookies.ipPort || "9090");
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [errorList, setErrorList] = useState<Array<ErrorItem>>([]);
  const [connected, setConnected] = useState(false);
  const [MovesList, setMovesListInternal] = useState(cookies.movesList || []);
  const [moving, setMoving] = useState(false);
  const [speechTarget, setSpeechTarget] = useState<Utterance>({
    text: "",
    metadata: null,
    fileLocation: null
  });
  const [speaking, setSpeaking] = useState(false);
  const [pose, setPose] = useState<JointState | null>(null);
  //const [poseListener, setPoseListener] = useState<ROSLIB.Topic | null>(null);

  // TODO: make this type more specific
  const setMovesList: SetMovesList = arg => {
    if (!moving) {
      setCookie("movesList", arg);
      setMovesListInternal(arg);
    }
  };

  const setIpAddrCook = (addr: string) => {
    setCookie("ipAddr", addr);
    setIpAddr(addr);
  };

  const setIpPortCook = (port: string) => {
    setCookie("ipPort", port);
    setIpPort(port);
  };

  const addError: AddError = (text, src) => {
    const newError = { text, time: new Date(), src };
    setErrorList([...errorList, newError]);
  };

  // TODO: TS
  const addToMoveList: AddToMoveList = value => {
    if (moving) {
      addError("Cannot add to moves list while moving", "core");
      return;
    }
    setMovesList([
      ...MovesList,
      {
        time: 2,
        pose: value,
        lr: "right",
        status: "not-run",
        key: genRandID()
      }
    ]);
  };

  const setConnectedWrap: SetConnected = con => {
    if (con === false && ros !== null) {
      setRos(null);
    }
    setConnected(con);
  };

  useEffect(() => {
    if (!connected) return;
    // TODO: Figure out how to clean up pose listener
    // poseListener.unsubscribe();
    // setPoseListener(null);
    const poseListenerT = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "joint_states",
      messageType: "sensor_msgs/JointState"
    });
    poseListenerT.subscribe(msg => {
      const cmsg = msg as JointState;
      if (cmsg.name.includes("left_shoulder_flexionextension")) {
        setPose(cmsg);
      }
    });
    //setPoseListener(poseListenerT);
  }, [connected, ros]);

  return (
    <CookiesProvider>
      <div className="App">
        <Header
          setRos={setRos}
          addError={addError}
          connected={connected}
          setConnected={setConnectedWrap}
          ipAddr={ipAddr}
          ipPort={ipPort}
          setIpAddr={setIpAddrCook}
          setIpPort={setIpPortCook}
        />
        <div className="body" style={{ backgroundColor: colors.gray.dark2 }}>
          <div
            className="visualFeeds"
            style={Object.assign({}, basicBlock, {
              maxWidth: "none",
              maxHeight: "auto",
              flexDirection: "row",
              flexWrap: "wrap"
            })}
          >
            <Vids
              ros={ros}
              connected={connected}
              ipAddr={ipAddr}
              ipPort={ipPort}
            />
            <URDF ros={ros} connected={connected} />
            <SystemMonitor ros={ros} connected={connected} />
          </div>
          <RelaxMotors ros={ros} connected={connected} />
          <div
            className="controls"
            style={{
              display: "flex",
              flexDirection: "row",
              flexWrap: "wrap",
              alignItems: "flex-start"
            }}
          >
            <Drive ros={ros} connected={connected} />
            <GameContainer ros={ros} connected={connected} />
            <div style={{ display: "flex", flexDirection: "row" }}>
              <SpeechContainer
                ros={ros}
                connected={connected}
                speechTarget={speechTarget}
                setSpeechTarget={setSpeechTarget}
                setSpeaking={setSpeaking}
                speaking={speaking}
              />
            </div>
            <FaceContainer ros={ros} connected={connected} />
            <div
              style={{
                display: "flex",
                flexDirection: "row",
                flexWrap: "wrap"
              }}
            >
              <PoseContainer
                ros={ros}
                connected={connected}
                addToMoveList={addToMoveList}
                pose={pose}
              />
              <SequenceRunContainer
                ros={ros}
                connected={connected}
                MovesList={MovesList}
                setMovesList={setMovesList}
                moving={moving}
                setMoving={setMoving}
              />
              <SequenceContainer
                ros={ros}
                connected={connected}
                MovesList={MovesList}
                setMovesList={setMovesList}
              />
            </div>
            <GameBuckets ros={ros} connected={connected} />
            <MoveToPose
              ros={ros}
              connected={connected}
              moving={moving}
              setMoving={setMoving}
              pose={pose}
            />
          </div>
          <ErrorDisplay errorList={errorList} />
        </div>
      </div>
    </CookiesProvider>
  );
};
export default RobotController;
