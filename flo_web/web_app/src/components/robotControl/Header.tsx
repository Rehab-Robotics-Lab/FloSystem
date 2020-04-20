import React, { useEffect } from "react";
import * as ROSLIB from "roslib";
import colors from "../../styleDefs/colors";
import { AddError, SetConnected } from "../robotController";
import { CookiesProvider } from "react-cookie";
import { useParams } from "react-router-dom";

interface HeaderProps {
  setRos: (ros: ROSLIB.Ros) => any;
  addError: AddError;
  connected: boolean;
  setConnected: SetConnected;
  ipAddr: string;
  ipPort: string;
  setIpAddr: (arg: string) => void;
  setIpPort: (arg: string) => void;
}

const Header: React.FunctionComponent<HeaderProps> = ({
  setRos,
  addError,
  connected,
  setConnected,
  ipAddr,
  ipPort,
  setIpAddr,
  setIpPort
}) => {
  const errorWrapper = (err: string): void => {
    addError("ROS Connection Error: " + err, "Header");
  };

  const { robotName } = useParams();

  //TODO: TS
  useEffect(() => {
    if (!ipAddr) return;
    if (connected === false) {
      const targUrl = `wss://${ipAddr}/robot/${robotName}`;
      const newRosConnection = new ROSLIB.Ros({
        url: targUrl
        //url: `ws://${ipAddr}:${ipPort}`
        //TODO: Obviously fix this up.
      });
      newRosConnection.on("error", err => {
        errorWrapper(err);
      });
      newRosConnection.on("connection", () => {
        console.log("connected to socket at: " + targUrl);
        setConnected(true);
      });
      newRosConnection.on("close", () => {
        setConnected(false);
      });
      setRos(newRosConnection);
    }
  }, [connected]);

  const connectedString = (): JSX.Element => {
    let toReturn: JSX.Element;
    if (connected) {
      toReturn = <span style={{ color: "green" }}>Connected</span>;
    } else {
      toReturn = <span style={{ color: "red" }}>Not Connected</span>;
    }
    return toReturn;
  };

  return (
    <CookiesProvider>
      <div
        style={{
          display: "flex",
          alignItems: "stretch",
          justifyContent: "space-between",
          backgroundColor: colors.blue.neutral,
          color: colors.white
        }}
      >
        <h1 style={{ margin: "0px" }}>Flo Control Center</h1>
        <div>
          <b>{connectedString()}</b>
        </div>
      </div>
    </CookiesProvider>
  );
};

export default Header;
