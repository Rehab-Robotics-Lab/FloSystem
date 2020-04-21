import React, { useEffect, useState } from "react";
import { Link, useHistory } from "react-router-dom";
import "./App.css";
import Routes from "./Routes";
import axios from "axios";
import { Helmet } from "react-helmet";

//function App() {
//return <RobotController />;
//}
//export default App;
function App() {
  const [loggedIn, setLoggedInInternal] = useState(false);
  const [userName, setUserName] = useState("");
  const [userType, setUserType] = useState<string>("");
  const history = useHistory();

  const setLoggedIn = (ut: boolean): void => {
    setLoggedInInternal(ut);
    if (ut === false) {
      history.push("/login");
    }
  };

  useEffect(() => {
    axios.get("/api/users/login").then(res => {
      const li = res.data["loggedIn"];
      setLoggedIn(li);
      if (li) {
        setUserName(res.data["userName"]);
        setUserType(res.data["userType"]);
      } else {
        history.push("/login");
      }
    });
  }, []);

  const navStyle: React.CSSProperties = {
    listStyleType: "none",
    margin: "0",
    padding: "0",
    overflow: "hidden",
    backgroundColor: "#333"
  };
  const linkStyle: React.CSSProperties = {
    display: "block",
    color: "white",
    textAlign: "center",
    padding: "14px 16px",
    textDecoration: "none",
    float: "left"
  };

  return (
    <div className="App container">
      <div>
        {loggedIn && "welcome " + userName}
        {loggedIn || "Not logged in"}
      </div>
      <nav style={navStyle}>
        <Link style={linkStyle} to="/">
          Home
        </Link>
        {userType === "administrator" && (
          <Link style={linkStyle} to="/admin">
            Admin Portal
          </Link>
        )}
        {loggedIn || (
          <Link style={linkStyle} to="/login">
            Login
          </Link>
        )}
        {loggedIn && (
          <a
            style={linkStyle}
            href="#"
            onClick={() => {
              axios.post("/api/users/logout").then(
                res => {
                  setLoggedIn(false);
                  setUserName("");
                },
                err => {
                  alert("failed to logout");
                }
              );
            }}
          >
            Logout
          </a>
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
