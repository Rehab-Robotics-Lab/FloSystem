import React, { useState, useEffect } from "react";
import "./App.css";
import { CookiesProvider, useCookies } from "react-cookie";
import Header from "./components/Header";
import URDF from "./components/urdf";
import PoseContainer from "./components/PoseContainer";
import ErrorDisplay, { ErrorItem } from "./components/ErrorDisplay";
import SequenceRunContainer, { Move } from "./components/SequenceRunContainer";
import SequenceContainer from "./components/SequenceContainer";
import colors from "./styleDefs/colors";
import SpeechContainer from "./components/SpeechContainer";
import SavedSpeech from "./components/SavedSpeech";
import MoveToPose from "./components/MoveToPose";
import FaceContainer from "./components/FaceContainer";
import RelaxMotors from "./components/RelaxMotors";
import * as ROSLIB from "roslib";

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

const App: React.FunctionComponent = () => {
  const [cookies, setCookie] = useCookies(["movesList"]);
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
  const [poseListener, setPoseListener] = useState<ROSLIB.Topic | null>(null);

  // TODO: make this type more specific
  const setMovesList: SetMovesList = arg => {
    if (!moving) {
      setCookie("movesList", arg);
      setMovesListInternal(arg);
    }
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
      setPose(msg as JointState);
    });
    setPoseListener(poseListenerT);
  }, [connected, ros]);

  return (
    <CookiesProvider>
      <div className="App">
        <Header
          setRos={setRos}
          addError={addError}
          connected={connected}
          setConnected={setConnectedWrap}
        />
        <div className="body" style={{ backgroundColor: colors.gray.dark2 }}>
          <div className="visualFeeds">
            <URDF ros={ros} connected={connected} />
            <RelaxMotors ros={ros} connected={connected} />
          </div>
          <div
            className="controls"
            style={{
              display: "flex",
              flexDirection: "row",
              flexWrap: "wrap",
              alignItems: "flex-start"
            }}
          >
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
export default App;
