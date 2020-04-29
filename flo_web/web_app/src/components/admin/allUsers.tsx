import React, { useState } from "react";
import axios from "axios";
import { SetInterval } from "../utilities";

interface User {
  id: number;
  first_name: string;
  last_name: string;
  email: string;
  last_login: Date;
  user_type: string;
}

const AllUsers: React.FunctionComponent = () => {
  const [userArray, setUserArray] = useState<Array<User>>([]);

  // Note this is coming from somewhere else, not vanilla
  SetInterval(() => {
    axios.get("/api/users/all-users").then(res => {
      setUserArray(res.data.users as Array<User>);
    });
  }, 1000 * 1);

  const userRows = userArray.map(user => (
    <tr key={user.email}>
      <td>{user.id}</td>
      <td>{user.first_name}</td>
      <td>{user.last_name}</td>
      <td>{user.email}</td>
      <td>{user.user_type}</td>
    </tr>
  ));

  return (
    <div>
      <h2>User List</h2>
      <table style={{ marginLeft: "auto", marginRight: "auto" }}>
        <thead>
          <tr>
            <th>ID</th>
            <th>First Name</th>
            <th>Last Name</th>
            <th>Email</th>
            <th>User Type</th>
          </tr>
        </thead>
        <tbody>{userRows}</tbody>
      </table>
    </div>
  );
};

export default AllUsers;
