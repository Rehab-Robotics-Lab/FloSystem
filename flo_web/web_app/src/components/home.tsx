import React from "react";
import Robots from "./robots";

interface HomeProps {
  userName: string;
  userType: string;
}

const Home: React.FunctionComponent<HomeProps> = ({ userName, userType }) => {
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
          If you have any trouble with the system, please reach out to{" "}
          <a href="https://michaelsobrepera.com/contact/">Michael Sobrepera</a>
        </p>
      </div>

      {userName && <Robots />}
    </div>
  );
};

export default Home;
