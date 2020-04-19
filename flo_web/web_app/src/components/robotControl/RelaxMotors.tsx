import React from "react";
import { majorButton } from "../../styleDefs/styles";
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
    console.log("sent command to relax motors");
  };

  return (
    <div
      style={Object.assign({}, majorButton, {
        width: "100%"
      })}
    >
      <button
        type="button"
        disabled={!connected}
        style={majorButton}
        onClick={(): void => relaxMotors()}
      >
        Relax Motors
      </button>
    </div>
  );
};

export default RelaxMotors;
