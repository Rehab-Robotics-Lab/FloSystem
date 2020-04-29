import React, { useState } from "react";
import axios from "axios";
import { SetInterval } from "../utilities";

interface Robot {
  id: number;
  ipaddr: string;
  robot_name: string;
  robot_type: string;
  connected: boolean;
  battery: number;
  last_login: Date;
  active_user_id: number;
  active_user_first: string;
  active_user_last: string;
  active_user_email: string;
}

const AllRobots: React.FunctionComponent = () => {
  const [robotArray, setRobotArray] = useState<Array<Robot>>([]);

  // Note this is coming from somewhere else, not vanilla
  SetInterval(() => {
    axios.get("/api/robots/all-robots").then(res => {
      setRobotArray(res.data.robots as Array<Robot>);
    });
  }, 1000 * 1);

  const robotRows = robotArray.map(robot => (
    <tr key={robot.robot_name}>
      <td>{robot["id"]}</td>
      <td>{robot["robot_name"]}</td>
      <td>{robot["connected"].toString()}</td>
      <td>{robot["ipaddr"]}</td>
      <td>{robot["robot_type"]}</td>
      <td>{robot["battery"]}</td>
      <td>
        {robot["active_user_first"] +
          " " +
          robot["active_user_last"] +
          " (" +
          robot["active_user_email"] +
          ")"}
      </td>
    </tr>
  ));

  return (
    <div>
      <h2>Robot List</h2>
      <table style={{ marginLeft: "auto", marginRight: "auto" }}>
        <thead>
          <tr>
            <th>ID</th>
            <th>Robot Name</th>
            <th>Connected</th>
            <th>IP Addr</th>
            <th>Type</th>
            <th>Battery</th>
            <th>Active User</th>
          </tr>
        </thead>
        <tbody>{robotRows}</tbody>
      </table>
    </div>
  );
};

export default AllRobots;
