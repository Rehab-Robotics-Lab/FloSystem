import React from "react";
import { Route, Switch } from "react-router-dom";
import RobotController from "./components/robotController";
import Login from "./components/users/login";
import Logout from "./components/users/logout";

export default function Routes() {
  return (
    <Switch>
      <Route exact path="/">
        <div> hi</div>
      </Route>
      <Route exact path="/controller">
        <RobotController />
      </Route>
      <Route exact path="/login">
        <Login />
      </Route>
    </Switch>
  );
}
