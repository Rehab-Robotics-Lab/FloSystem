import React, { useState } from "react";
import "./App.css";
import { CookiesProvider, useCookies } from "react-cookie";
import Header from "./components/Header";
import URDF from "./components/urdf";
import PoseContainer from "./components/PoseContainer";
import ErrorDisplay, { ErrorItem } from "./components/ErrorDisplay";
import SequenceRunContainer, { Move } from "./components/SequenceRunContainer";
import SequenceContainer from "./components/SequenceContainer";
import colors from "./styleDefs/colors";

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

const App: React.FunctionComponent = () => {
  const [cookies, setCookie] = useCookies(["movesList"]);
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [errorList, setErrorList] = useState<Array<ErrorItem>>([]);
  const [connected, setConnected] = useState(false);
  const [MovesList, setMovesListInternal] = useState(cookies.movesList || []);
  const [moving, setMoving] = useState(false);

  // TODO: make this type more specific
  const setMovesList: SetMovesList = arg => {
    if (!moving) {
      setCookie("movesList", arg);
      setMovesListInternal(arg);
    }
  };

  const addError: AddError = (text, src) => {
    const newError = { text, time: new Date(), src };
    // setErrorList([...errorList, newError]); TODO: get this back in
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
          </div>
          <div style={{ display: "flex", flexDirection: "row" }}>
            <PoseContainer
              ros={ros}
              connected={connected}
              addToMoveList={addToMoveList}
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
          <ErrorDisplay errorList={errorList} />
        </div>
      </div>
    </CookiesProvider>
  );
};
export default App;
