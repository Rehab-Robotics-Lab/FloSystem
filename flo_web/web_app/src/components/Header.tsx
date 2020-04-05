import React from "react";
import * as ROSLIB from "roslib";
import colors from "../styleDefs/colors";
import { AddError, SetConnected } from "../App";
import { CookiesProvider } from "react-cookie";

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

  //TODO: TS
  const handleSubmit = (e: any) => {
    e.preventDefault();
    if (!(ipAddr && ipPort)) return;
    const newRosConnection = new ROSLIB.Ros({
      url: `ws://${ipAddr}:${ipPort}/robot/1`
      //TODO: Obviously fix this up.
    });
    newRosConnection.on("error", err => {
      errorWrapper(err);
    });
    newRosConnection.on("connection", () => {
      setConnected(true);
    });
    newRosConnection.on("close", () => {
      setConnected(false);
    });
    setRos(newRosConnection);
  };

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
          <form onSubmit={handleSubmit}>
            <label htmlFor="ip_addr">
              IP Address:
              <input
                type="text"
                name="ip_addr"
                value={ipAddr}
                onChange={(e): void => {
                  setIpAddr(e.target.value);
                }}
              />
            </label>
            <label htmlFor="ip_port">
              IP Port:
              <input
                type="text"
                name="ip_port"
                value={ipPort}
                onChange={(e): void => {
                  setIpPort(e.target.value);
                }}
              />
            </label>
            <input type="submit" value="Connect" disabled={connected} />
          </form>
          <b>{connectedString()}</b>
        </div>
      </div>
    </CookiesProvider>
  );
};

export default Header;
