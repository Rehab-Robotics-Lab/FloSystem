import React from "react";
import { Route, Switch } from "react-router-dom";
import { useHistory, Link, useRouteMatch } from "react-router-dom";
import AddRobot from "./addRobot";

interface AdminProps {
  loggedIn: boolean;
  userType: string;
}

const Admin: React.FunctionComponent<AdminProps> = ({ loggedIn, userType }) => {
  const history = useHistory();

  const { path, url } = useRouteMatch();

  if (userType !== "administrator") {
    history.push("/");
  }
  //<Route path="/user/:userEmail">
  //<RobotController />
  //</Route>
  return (
    <div>
      <h1>Admin Portal</h1>

      <Link to={`${url}/add-robot`}>Add Robot</Link>

      <Switch>
        <Route exact path={`${path}/add-robot`}>
          <AddRobot />
        </Route>
      </Switch>
    </div>
  );
};

export default Admin;
