import React, { useState } from "react";
import axios from "axios";
import { Link } from "react-router-dom";
import { SetInterval } from "./utilities";

interface Robot {
  robot_name: string;
  connected: boolean;
  battery: number;
  active_user_id: number;
  active_user_first: string;
  active_user_last: string;
}

const Robots: React.FunctionComponent = () => {
  const [robotArray, setRobotArray] = useState<Array<Robot>>([]);

  // Note this is coming from somewhere else, not vanilla
  SetInterval(() => {
    axios.get("/api/robots/").then(res => {
      setRobotArray(res.data.robots as Array<Robot>);
    });
  }, 1000 * 1);

  const robotRows = robotArray.map(robot => {
    const canConnect = robot["active_user_first"] == null && robot["connected"];
    return (
      <tr key={robot.robot_name}>
        <td>{robot["robot_name"]}</td>
        <td>{robot["connected"].toString()}</td>
        <td>{robot["battery"]}</td>
        <td>{robot["active_user_first"]}</td>
        <td>
          {canConnect && (
            <Link to={`/controller/${robot["robot_name"]}`}>Connect</Link>
          )}
          {canConnect || "Not Available"}
        </td>
      </tr>
    );
  });

  return (
    <div>
      <h2>Robot List</h2>
      <table style={{ marginLeft: "auto", marginRight: "auto" }}>
        <thead>
          <tr>
            <th>Robot Name</th>
            <th>Connected</th>
            <th>Battery</th>
            <th>Active User</th>
            <th>Connect</th>
          </tr>
        </thead>
        <tbody>{robotRows}</tbody>
      </table>
    </div>
  );
};

export default Robots;
