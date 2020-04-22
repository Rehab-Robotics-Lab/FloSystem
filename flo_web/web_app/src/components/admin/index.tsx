import React from "react";
import { Route, Switch } from "react-router-dom";
import { useHistory, Link, useRouteMatch } from "react-router-dom";
import AddRobot from "./addRobot";
import ChangePermissions from "./changePermissions";
import ResetUserPassword from "./resetUserPassword";
import AllRobots from "./allRobots";
import AllUsers from "./allUsers";
import ChangeUserType from "./changeUserType";
import ResetRobotPassword from "./resetRobotPassword";

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

      <ul style={{ listStyleType: "none" }}>
        <li>
          <Link to={`${url}/all-robots`}>List All Robots</Link>
        </li>
        <li>
          <Link to={`${url}/add-robot`}>Add Robot</Link>
        </li>
        <li>
          <Link to={`${url}/reset-robot-password`}>Reset Robot password</Link>
        </li>
        <li>
          <Link to={`${url}/all-users`}>List All Users</Link>
        </li>
        <li>
          <Link to={`${url}/change-permissions`}>Change Permissions</Link>
        </li>
        <li>
          <Link to={`${url}/reset-user-password`}>Reset User Password</Link>
        </li>
        <li>
          <Link to={`${url}/change-user-type`}>Change User Type</Link>
        </li>
      </ul>

      <Switch>
        <Route exact path={`${path}/add-robot`}>
          <AddRobot />
        </Route>
        <Route exact path={`${path}/change-permissions`}>
          <ChangePermissions />
        </Route>
        <Route exact path={`${path}/reset-user-password`}>
          <ResetUserPassword />
        </Route>
        <Route exact path={`${path}/all-robots`}>
          <AllRobots />
        </Route>
        <Route exact path={`${path}/all-users`}>
          <AllUsers />
        </Route>
        <Route exact path={`${path}/change-user-type`}>
          <ChangeUserType />
        </Route>
        <Route exact path={`${path}/reset-robot-password`}>
          <ResetRobotPassword />
        </Route>
      </Switch>
    </div>
  );
};

export default Admin;
