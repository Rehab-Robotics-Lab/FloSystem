import React, { useEffect, useState } from "react";
import { Link, useHistory } from "react-router-dom";
import "./App.css";
import Routes from "./Routes";
import axios from "axios";

//function App() {
//return <RobotController />;
//}
//export default App;
function App() {
  const [loggedIn, setLoggedInInternal] = useState(false);
  const [userName, setUserName] = useState("");
  const [userType, setUserType] = useState("standard");
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
      } else {
        history.push("/login");
      }
    });
  }, []);

  return (
    <div className="App container">
      <div>
        {loggedIn && "welcome " + userName}
        {loggedIn || "Not logged in"}
      </div>
      <nav>
        <Link to="/">Home</Link>
        <Link to="/controller">Controller</Link>
        {loggedIn || <Link to="/login">Login</Link>}
        {loggedIn && (
          <a
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
              console.log("logout");
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
      />
    </div>
  );
}

export default App;
