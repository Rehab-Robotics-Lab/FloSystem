import React, { useEffect, useState, useCallback } from "react";
import { NavLink, useHistory } from "react-router-dom";
import "./App.css";
import Routes from "./Routes";
import axios from "axios";
import navbar from "./styleDefs/Nav.module.css";
import Honeybadger from "honeybadger-js";

//function App() {
//return <RobotController />;
//}
//export default App;
const App: React.FunctionComponent = () => {
  const [loggedIn, setLoggedInInternal] = useState(false);
  const [userName, setUserNameInternal] = useState("");
  const [userType, setUserType] = useState<string>("");
  const history = useHistory();

  const setUserName = useCallback(
    (un: string): void => {
      setUserNameInternal(un);
      Honeybadger.setContext({
        //eslint-disable-next-line @typescript-eslint/camelcase
        user_email: un
      });
    },
    [setUserNameInternal]
  );

  const setLoggedIn = useCallback(
    (ut: boolean): void => {
      setLoggedInInternal(ut);
      if (ut === false) {
        history.push("/login");
      }
    },
    [setLoggedInInternal, history]
  );

  useEffect(() => {
    axios.get("/api/users/login").then(res => {
      const li = res.data["loggedIn"];
      setLoggedIn(li);
      if (li) {
        setUserName(res.data["userName"]);
        setUserType(res.data["userType"]);
      }
    });
  }, [setLoggedIn, setUserName, setUserType]);

  return (
    <div className="App container">
      <div>
        {loggedIn && "welcome " + userName}
        {loggedIn || "Not logged in"}
      </div>
      <nav className={navbar.navbar}>
        <NavLink
          className={navbar.navitem}
          exact={true}
          activeClassName={navbar.activenavitem}
          to="/"
        >
          Home
        </NavLink>
        {userType === "administrator" && (
          <NavLink
            className={navbar.navitem}
            activeClassName={navbar.activenavitem}
            to="/admin"
          >
            Admin Portal
          </NavLink>
        )}
        {loggedIn || (
          <NavLink
            className={navbar.navitem}
            activeClassName={navbar.activenavitem}
            to="/login"
          >
            Login
          </NavLink>
        )}
        {loggedIn && (
          <NavLink
            className={navbar.navitem}
            activeClassName={navbar.activenavitem}
            to="/change-password"
          >
            Change Password
          </NavLink>
        )}
        {loggedIn && (
          <button
            className={navbar.navitem}
            onClick={(): void => {
              axios.post("/api/users/logout").then(
                () => {
                  // request succeeded
                  setLoggedIn(false);
                  setUserName("");
                },
                err => {
                  alert("failed to logout: " + err.response.data["error"]);
                }
              );
            }}
          >
            Logout
          </button>
        )}
      </nav>
      <Routes
        loggedIn={loggedIn}
        setLoggedIn={setLoggedIn}
        setUserName={setUserName}
        setUserType={setUserType}
        userType={userType}
        userName={userName}
      />
    </div>
  );
};

export default App;
