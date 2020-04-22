import React from "react";
import colors from "../../styleDefs/colors";
import { CookiesProvider } from "react-cookie";

interface HeaderProps {
  connected: boolean;
}

const Header: React.FunctionComponent<HeaderProps> = ({ connected }) => {
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
