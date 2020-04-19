import React from "react";
import { Link } from "react-router-dom";
import "./App.css";
import Routes from "./Routes";

//function App() {
//return <RobotController />;
//}
//export default App;
function App() {
  return (
    <div className="App container">
      <nav>
        <Link to="/">Home</Link>
        <Link to="/controller">Controller</Link>
      </nav>
      <Routes />
    </div>
  );
}

export default App;
