import React from "react";
import { Route, Switch } from "react-router-dom";
import { useHistory, Link } from "react-router-dom";

interface HomeProps {
  userName: string;
  userType: string;
}

const Home: React.FunctionComponent<HomeProps> = ({ userName, userType }) => {
  return (
    <div>
      <h1>Welcome!!</h1>
      <p>
        Welcome to the Flo Robots Control Center. Here you can access and
        control robots.
      </p>
      <ul>
        <li>You are logged in as: {userName}</li>
        <li>You are a: {userType} type user</li>
      </ul>
      <p>
        If you have any trouble with the system, please rech out to{" "}
        <a href="https://michaelsobrepera.com/contact/">Michael Sobrepera</a>
      </p>
    </div>
  );
};

export default Home;
