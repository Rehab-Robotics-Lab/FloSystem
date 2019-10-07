import React from "react";
import { basicBlock, majorButton } from "../styleDefs/styles";
import * as ROSLIB from "roslib";

interface RelaxMotorsProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}

// Takes a parameter ros, which is the connection to ros
const RelaxMotors: React.FunctionComponent<RelaxMotorsProps> = ({
  ros,
  connected
}) => {
  const relaxMotors = (): void => {
    if (!connected || ros === null) {
      return;
    }
    const topic = new ROSLIB.Topic({
      ros: ros,
      name: "/motor_commands",
      messageType: "std_msgs/String"
    });

    const relax = new ROSLIB.Message({
      data: "relax"
    });
    topic.publish(relax);
  };

  return (
    <div style={basicBlock}>
      <button
        type="button"
        disabled={!connected}
        style={majorButton}
        onClick={() => relaxMotors()}
      >
        Relax Motors
      </button>
    </div>
  );
};

export default RelaxMotors;
