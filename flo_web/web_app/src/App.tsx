import React, { useEffect, useState, useCallback } from "react";
import { NavLink, useHistory } from "react-router-dom";
import "./App.css";
import Routes from "./Routes";
import axios from "axios";
import navbar from "./styleDefs/Nav.module.css";

//function App() {
//return <RobotController />;
//}
//export default App;
function App() {
  const [loggedIn, setLoggedInInternal] = useState(false);
  const [userName, setUserName] = useState("");
  const [userType, setUserType] = useState<string>("");
  const history = useHistory();

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
            onClick={() => {
              axios.post("/api/users/logout").then(
                res => {
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
}

export default App;
