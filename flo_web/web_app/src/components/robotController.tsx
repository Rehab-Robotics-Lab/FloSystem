import React, { useReducer, useState, useEffect, useCallback } from "react";
import "../App.css";
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
import { useParams, useHistory } from "react-router-dom";
import { PoseWrapper } from "./robotControl/seq_pose/PoseContainer";
import EventEmitter2 from "eventemitter2";

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
  (value: PoseWrapper): void;
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

const ipAddr = window.location.hostname;

const RobotController: React.FunctionComponent = () => {
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  //const [errorList, setErrorList] = useState<Array<ErrorItem>>([]);
  function errorReducer(
    errorList: ErrorItem[],
    newError: { text: string; src: string }
  ): ErrorItem[] {
    const err = {
      text: newError.text,
      time: new Date(),
      src: newError.src
    };
    errorList = [...errorList, err];
    return errorList;
  }

  const [errorList, addError] = useReducer(errorReducer, []);
  const [connected, setConnected] = useState(false);

  const [MovesList, setMovesListInternal] = useState<Move[]>([]);
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
      setMovesListInternal(arg);
    }
  };

  // TODO: TS
  const addToMoveList: AddToMoveList = (value: PoseWrapper) => {
    if (moving) {
      addError({ text: "Cannot add to moves list while moving", src: "core" });
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

  const setConnectedWrap: SetConnected = useCallback(
    con => {
      if (con === false) {
        setRos(null);
      }
      setConnected(con);
    },
    [setConnected, setRos]
  );

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
    const plCB = (msg: ROSLIB.Message): void => {
      const cmsg = msg as JointState;
      if (cmsg.name.includes("left_shoulder_flexionextension")) {
        setPose(cmsg);
      }
    };
    poseListenerT.subscribe(plCB);

    return (): void => {
      poseListenerT.unsubscribe(plCB);
    };
    //setPoseListener(poseListenerT);
  }, [connected, ros]);

  const { robotName } = useParams();
  const history = useHistory();

  const goHome = useCallback((): void => {
    console.log("*** Going Home ***");
    history.push("/");
  }, [history]);

  useEffect(() => {
    const targUrl = `wss://${ipAddr}/robot/${robotName}`;
    const newRosConnection = new ROSLIB.Ros({
      url: targUrl
    });
    newRosConnection.on("error", err => {
      addError({ text: "ROS Connection Error: " + err, src: "Header" });
    });
    newRosConnection.on("connection", () => {
      console.log("connected to socket at: " + targUrl);
      setConnectedWrap(true);
    });
    newRosConnection.on("close", () => {
      setConnectedWrap(false);
      goHome();
    });
    setRos(newRosConnection);

    return (): void => {
      console.log("******CLOSING ROS CONNECTION********");
      newRosConnection.close();
      ((newRosConnection as unknown) as EventEmitter2.EventEmitter2).removeAllListeners();
      setConnectedWrap(false);
      goHome();
    };
  }, [addError, goHome, robotName, setConnectedWrap, setRos]);
  //}, [connected]);

  return (
    <div className="App">
      <div
        className="body"
        style={{
          backgroundColor: colors.gray.dark2,
          display: "flex",
          width: "100%",
          height: "100%",

          alignItems: "flex-start",
          margin: "0px",
          flexDirection: "column"
        }}
      >
        <div
          className="visualFeeds"
          style={Object.assign({}, basicBlock, {
            maxWidth: "none",
            maxHeight: "auto",
            flexDirection: "row",
            flexWrap: "wrap",
            position: "sticky",
            top: 0,
            justifyContent: "spaceAround"
          })}
        >
          <Vids ros={ros} connected={connected} ipAddr={ipAddr} />
          <URDF ros={ros} connected={connected} />
        </div>
        <div style={{ flexGrow: 1, overflow: "auto" }}>
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
    </div>
  );
};
export default RobotController;
