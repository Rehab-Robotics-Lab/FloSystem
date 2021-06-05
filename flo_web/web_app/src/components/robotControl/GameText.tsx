import React, { useState, useEffect } from "react";
import * as ROSLIB from "roslib";

interface GameTextMsg {
  data: string;
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

    return (): void => {
      GameTextListener.unsubscribe(gtCallback);
    };
  }, [connected, ros]);
  return (
    <div
      style={{
        height: "40px",
        overflow: "auto",
        display: "flex",
        flexDirection: "column-reverse",
        color: "red",
      }}
    >
      {latestGameText}
    </div>
  );
};

export default GameText;
