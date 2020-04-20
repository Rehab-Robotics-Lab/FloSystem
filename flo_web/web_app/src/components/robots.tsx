import React, { useEffect, useState } from "react";
import axios from "axios";
import { useHistory, Link } from "react-router-dom";
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
  const history = useHistory();
  const [robotArray, setRobotArray] = useState<Array<Robot>>([]);

  SetInterval(() => {
    axios.get("/api/robots/").then(res => {
      setRobotArray(res.data.robots as Array<Robot>);
    });
  }, 1000 * 1);

  const robotRows = robotArray.map(robot => (
    <tr key={robot.robot_name}>
      <th>{robot["robot_name"]}</th>
      <th>{robot["connected"]}</th>
      <th>{robot["battery"]}</th>
      <th>{robot["active_user_first"]}</th>
      <th>connect</th>
    </tr>
  ));

  return (
    <div>
      <h2>Robot List</h2>
      <table>
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
