import React, { useState, useEffect } from "react";
import * as ROSLIB from "roslib";
import { basicBlock } from "../../styleDefs/styles";

interface CommandOpts {
  options: string[];
}

interface GameState {
  state: string;
}

interface GameCommandProps {
  name: string;
  run: () => void;
}

const GameCommand: React.FunctionComponent<GameCommandProps> = ({
  name,
  run
}) => {
  return (
    <button
      type="button"
      style={{ wordWrap: "break-word" }}
      onClick={(): void => {
        run();
      }}
    >
      {name}
    </button>
  );
};

interface GameDefProps {
  name: string;
  run: () => void;
  disabled: boolean;
}

const GameDef: React.FunctionComponent<GameDefProps> = ({
  name,
  run,
  disabled
}) => {
  return (
    <button
      type="button"
      disabled={disabled}
      onClick={(): void => {
        run();
      }}
    >
      {name}
    </button>
  );
};

interface GameContainerProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}
// Takes a parameter ros, which is the connection to ros
const GameContainer: React.FunctionComponent<GameContainerProps> = ({
  ros,
  connected
}) => {
  const [commandOptions, setCommandOptions] = useState<string[]>([]);
  const [gameFeedback, setGameFeedback] = useState<string>("");
  const [gameDefPub, setGameDefPub] = useState<ROSLIB.Topic | null>(null);
  const [gameCommandPub, setGameCommandPub] = useState<ROSLIB.Topic | null>(
    null
  );

  useEffect(() => {
    if (!connected) return;

    const CommandListener = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "game_runner_command_opts",
      messageType: "flo_core_defs/GameCommandOptions"
    });
    const clCallback = (msg: ROSLIB.Message): void => {
      setCommandOptions((msg as CommandOpts).options);
      console.log("got new command options");
    };
    CommandListener.subscribe(clCallback);
    console.log("subscribed to command options topic");

    const FeedbackListener = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "game_runner_state",
      messageType: "flo_core_defs/GameState"
    });
    const flCallback = (msg: ROSLIB.Message): void => {
      setGameFeedback((msg as GameState).state);
      console.log("got new game feedback");
    };
    FeedbackListener.subscribe(flCallback);
    console.log("subscribed to game feedback topic");

    const gameDefPubT = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "/game_runner_def",
      messageType: "flo_core_defs/GameDef"
    });
    setGameDefPub(gameDefPubT);
    console.log("connected to publish on game runner definition topic");

    const gameCommandPubT = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "/game_runner_commands",
      messageType: "flo_core_defs/GameCommand"
    });
    setGameCommandPub(gameCommandPubT);
    console.log("connected to publish on game runner commands topic");

    return (): void => {
      CommandListener.unsubscribe(clCallback);
      FeedbackListener.unsubscribe(flCallback);
      gameDefPubT.unadvertise();
      gameCommandPubT.unadvertise();
    };
  }, [connected, ros]);

  const availableGames = ["simon_says", "target_touch"];

  //{availableGames.map(value => (
  //<GameDef
  //name={value}
  //disabled={gameFeedback!=='waiting_for_def' || gameDefPub===null}
  //run={()=>{
  //let game_def = new ROSLIB.Message({
  //gameType:{value}
  //})
  //if (gameDefPub !==null){
  //gameDefPub.publish(game_def)
  //}}}
  ///>

  //))}

  return (
    <div
      style={Object.assign({}, basicBlock, {
        maxwidth: "400px"
      })}
    >
      <h2>Games:</h2>

      <button
        type="button"
        disabled={gameFeedback !== "waiting_for_def" || gameDefPub === null}
        onClick={(): void => {
          const simonSaysDef = new ROSLIB.Message({
            game_type: "simon_says" // eslint-disable-line
          });
          if (gameDefPub !== null) {
            gameDefPub.publish(simonSaysDef);
            console.log("sent command to play simon says");
          } else {
            console.error("not able to publish game def");
          }
        }}
      >
        Simon Says
      </button>

      <button
        type="button"
        disabled={gameFeedback !== "waiting_for_def" || gameDefPub === null}
        onClick={(): void => {
          const simonSaysDef = new ROSLIB.Message({
            game_type: "target_touch" // eslint-disable-line
          });
          if (gameDefPub !== null) {
            gameDefPub.publish(simonSaysDef);
            console.log("sent command to play target touch");
          } else {
            console.error("not able to publish game def");
          }
        }}
      >
        Target Touch
      </button>

      {commandOptions.map(value => (
        <GameCommand
          name={value}
          run={(): void => {
            const msg = new ROSLIB.Message({
              command: value
            });
            if (gameCommandPub !== null) {
              gameCommandPub.publish(msg);
            }
          }}
          key={value}
        />
      ))}
    </div>
  );
};

export default GameContainer;
