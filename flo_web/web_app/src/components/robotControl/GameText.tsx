import React, { useState, useEffect } from "react";
import * as ROSLIB from "roslib";

interface GameTextMsg {
  data: string;
}
interface HumanoidMsg {
  data: boolean;
}

interface GameTextProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}

const GameText: React.FunctionComponent<GameTextProps> = ({
  ros,
  connected,
}) => {
  const [latestGameText, setLatestGameText] = useState<string>("");
  const [humanoid, setHumanoid] = useState<boolean>(false);
  useEffect(() => {
    if (!connected) return;

    const GameTextListener = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "game_runner_text",
      messageType: "std_msgs/String",
    });
    const gtCallback = (msg: ROSLIB.Message): void => {
      setLatestGameText((msg as GameTextMsg).data);
      console.log("got new game text");
    };
    GameTextListener.subscribe(gtCallback);
    console.log("subscribed to game text");

    const HumanoidListener = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "humanoid_connnection_change",
      messageType: "std_msgs/Bool",
    });
    const hCallback = (msg: ROSLIB.Message): void => {
      setHumanoid((msg as HumanoidMsg).data);
      console.log("got new humanoid state");
    };
    HumanoidListener.subscribe(hCallback);
    console.log("subscribed to humanoid updates");

    const HumanoidParam = new ROSLIB.Param({
      ros: ros as ROSLIB.Ros,
      name: "humanoid",
    });
    HumanoidParam.get((value) => {
      setHumanoid(value);
      console.log("set humanoid to: " + value);
    });

    return (): void => {
      GameTextListener.unsubscribe(gtCallback);
      HumanoidListener.unsubscribe(gtCallback);
    };
  }, [connected, ros]);
  return (
    <div
      style={{
        height: "40px",
        overflow: "auto",
        display: humanoid ? "none" : "flex",
        flexDirection: "column-reverse",
        color: "red",
      }}
    >
      {latestGameText}
    </div>
  );
};

export default GameText;
