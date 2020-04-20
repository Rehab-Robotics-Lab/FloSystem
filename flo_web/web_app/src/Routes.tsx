import React from "react";
import { Route, Switch } from "react-router-dom";
import RobotController from "./components/robotController";
import Login from "./components/users/login";
import Register from "./components/users/register";
import Robots from "./components/robots";

interface RoutesProps {
  loggedIn: boolean;
  setLoggedIn: (arg: boolean) => void;
  setUserName: (arg: string) => void;
  setUserType: (arg: string) => void;
}

const Routes: React.FunctionComponent<RoutesProps> = ({
  loggedIn,
  setLoggedIn,
  setUserName,
  setUserType
}) => {
  return (
    <Switch>
      <Route exact path="/">
        <div> hi</div>
      </Route>
      <Route path="/controller/:robotName">
        <RobotController />
      </Route>
      <Route exact path="/login">
        <Login
          setLoggedIn={setLoggedIn}
          setUserName={setUserName}
          setUserType={setUserType}
        />
      </Route>
      <Route exact path="/register">
        <Register />
      </Route>
      <Route exact path="/robots">
        <Robots />
      </Route>
    </Switch>
  );
};

export default Routes;
