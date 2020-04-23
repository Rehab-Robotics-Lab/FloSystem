import React from "react";
import Robots from "./robots";
import { useHistory } from "react-router-dom";

interface HomeProps {
  userName: string;
  userType: string;
}

const Home: React.FunctionComponent<HomeProps> = ({ userName, userType }) => {
  const history = useHistory();
  return (
    <div>
      <div>
        <h1>Welcome!!</h1>
        <p>
          Welcome to the Flo Robots Control Center. Here you can access and
          control robots.
        </p>
        <ul style={{ listStyleType: "none" }}>
          <li>You are logged in as: {userName}</li>
          <li>Your user type is: {userType}</li>
        </ul>
        <p>
          If you have any trouble with the system, please rech out to{" "}
          <a href="https://michaelsobrepera.com/contact/">Michael Sobrepera</a>
        </p>
      </div>

      <Robots />
    </div>
  );
};

export default Home;
