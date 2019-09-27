import React, { useState } from "react";
import * as ROSLIB from "roslib";
import colors from "../styleDefs/colors";
import { AddError, SetConnected } from "../App";
import { CookiesProvider, useCookies } from "react-cookie";

interface HeaderProps {
  setRos: (ros: ROSLIB.Ros) => any;
  addError: AddError;
  connected: boolean;
  setConnected: SetConnected;
}

const Header: React.FunctionComponent<HeaderProps> = ({
  setRos,
  addError,
  connected,
  setConnected
}) => {
  const [cookies, setCookie] = useCookies(["ipAddr", "ipPort"]);
  const [ipAddr, setIpAddr] = useState(
    cookies.ipAddr || window.location.hostname
  );
  const [ipPort, setIpPort] = useState(cookies.ipPort || "9090");

  const errorWrapper = (err: string) => {
    addError("ROS Connection Error: " + err, "Header");
  };

  //TODO: TS
  const handleSubmit = (e: any) => {
    e.preventDefault();
    if (!(ipAddr && ipPort)) return;
    const newRosConnection = new ROSLIB.Ros({
      url: `ws://${ipAddr}:${ipPort}`
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

  const connectedString = () => {
    let toReturn;
    if (!connected) {
      toReturn = <span style={{ color: "red" }}>Not Connected</span>;
    }
    if (connected) {
      toReturn = <span style={{ color: "green" }}>Connected</span>;
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
                onChange={e => {
                  setCookie("ipAddr", e.target.value);
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
                onChange={e => {
                  setCookie("ipPort", e.target.value);
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
