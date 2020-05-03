import React, { useReducer, useState, useEffect, useCallback } from "react";
import "../App.css";
import ErrorDisplay, { ErrorItem } from "./robotControl/ErrorDisplay";
import colors from "../styleDefs/colors";
import Vids from "./robotControl/simpleVids";
import * as ROSLIB from "roslib";
import Drive from "./robotControl/Drive";
import { basicBlock } from "../styleDefs/styles";
import SystemMonitor from "./robotControl/SystemMonitor";
import EventEmitter2 from "eventemitter2";
import { SetConnected } from "./robotController";
import { useParams, useHistory } from "react-router-dom";

const ipAddr = window.location.hostname;

const SimpleRobotController: React.FunctionComponent = () => {
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

  const setConnectedWrap: SetConnected = useCallback(
    con => {
      if (con === false) {
        setRos(null);
      }
      setConnected(con);
    },
    [setConnected, setRos]
  );

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

          alignItems: "center",
          margin: "0px",
          flexDirection: "column",
          justifyContent: "center"
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
            justifyContent: "space-around"
          })}
        >
          <Vids ros={ros} connected={connected} ipAddr={ipAddr} />
        </div>
        <div style={{ flexGrow: 1, overflow: "auto" }}>
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
          </div>
        </div>
        <ErrorDisplay errorList={errorList} />
      </div>
    </div>
  );
};
export default SimpleRobotController;
