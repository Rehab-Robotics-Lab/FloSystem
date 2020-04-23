import React from "react";
import { Route, Switch } from "react-router-dom";
import RobotController from "./components/robotController";
import Login from "./components/users/login";
import Register from "./components/users/register";
import Admin from "./components/admin";
import Home from "./components/home";
import ChangePassword from "./components/users/changePassword";

interface RoutesProps {
  loggedIn: boolean;
  setLoggedIn: (arg: boolean) => void;
  setUserName: (arg: string) => void;
  setUserType: (arg: string) => void;
  userType: string;
  userName: string;
}

const Routes: React.FunctionComponent<RoutesProps> = ({
  loggedIn,
  setLoggedIn,
  setUserName,
  setUserType,
  userType,
  userName
}) => {
  return (
    <Switch>
      <Route exact path="/">
        <Home userName={userName} userType={userType} />
      </Route>
      <Route path="/controller/:robotName">
        <RobotController />
      </Route>
      <Route exact path="/login">
        <Login
          setLoggedIn={setLoggedIn}
          setUserName={setUserName}
          setUserType={setUserType}
          loggedIn={loggedIn}
        />
      </Route>
      <Route exact path="/register">
        <Register />
      </Route>
      <Route path="/admin">
        <Admin loggedIn={loggedIn} userType={userType} />
      </Route>
      <Route path="/change-password">
        <ChangePassword />
      </Route>
    </Switch>
  );
};

export default Routes;
