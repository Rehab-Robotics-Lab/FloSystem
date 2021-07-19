import React, { useState, useEffect } from "react";
import * as ROSLIB from "roslib";
import { basicBlock } from "../../styleDefs/styles";
import ModalWrapper from "./ModalWrapper";
import { GameBucket, StepDef } from "./GameBuckets";

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
  run,
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

interface GameContainerProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}
// Takes a parameter ros, which is the connection to ros
const GameContainer: React.FunctionComponent<GameContainerProps> = ({
  ros,
  connected,
}) => {
  const [commandOptions, setCommandOptions] = useState<string[]>([]);
  const [gameFeedback, setGameFeedback] = useState<string>("");
  const [reps, setReps] = useState(3);
  const [maxSteps, setMaxSteps] = useState(4);
  const [minSteps, setMinSteps] = useState(2);
  const [bimanual, setBimanual] = useState(true);
  const [gameDefPub, setGameDefPub] = useState<ROSLIB.Topic | null>(null);
  const [gameCommandPub, setGameCommandPub] = useState<ROSLIB.Topic | null>(
    null
  );

  useEffect(() => {
    if (!connected) return;

    const CommandListener = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "game_runner_command_opts",
      messageType: "flo_core_defs/GameCommandOptions",
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
      messageType: "flo_core_defs/GameState",
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
      messageType: "flo_core_defs/GameDef",
    });
    setGameDefPub(gameDefPubT);
    console.log("connected to publish on game runner definition topic");

    const gameCommandPubT = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "/game_runner_commands",
      messageType: "flo_core_defs/GameCommand",
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

  const [showSelector, setShowSelector] = useState(false);
  const [gbID, setGbID] = useState(0);
  const [gameType, setGameType] = useState<"simon_says" | "target_touch" |"stream">(
    "simon_says"
  );
  const [buckets, setBuckets] = useState<GameBucket[]>([]);

  const startButton = (
    type: "simon_says" | "target_touch"| "stream",
    cleanText: string
  ): JSX.Element => {
    return (
      <button
        type="button"
        disabled={gameFeedback !== "waiting_for_def" || gameDefPub === null}
        onClick={(): void => {
          if (!connected) {
            return;
          }
          setGameType(type);
          const srv = new ROSLIB.Service({
            ros: ros as ROSLIB.Ros,
            name: "/search_game_bucket_name_desc",
            serviceType: "flo_core_defs/SearchGameBucket",
          });
          console.log("Connected to service to search for a game bucket");
          const request = new ROSLIB.ServiceRequest({ search: "" });
          //TODO: Should be sorting out the buckets by game type
          srv.callService(request, (resp) => {
            const tmpBuckets = [];
            for (let idx = 0; idx < resp["ids"].length; idx++) {
              tmpBuckets[resp["ids"][idx]] = resp["game_buckets"][idx];
            }
            setBuckets(tmpBuckets);
            setGbID(
              tmpBuckets.findIndex((arg) => {
                if (arg === undefined || arg.targeted_game != type) {
                  return false;
                } else {
                  return true;
                }
              })
            );
          });
          setShowSelector(true);
        }}
      >
        {cleanText}
      </button>
    );
  };

  return (
    <div
      style={Object.assign({}, basicBlock, {
        maxwidth: "400px",
      })}
    >
      <h2>Games:</h2>
      {startButton("simon_says", "Simon Says")}
      {startButton("target_touch", "Target Touch")}
      {startButton("stream", "Stream")}
      {commandOptions.map((value) => (
        <GameCommand
          name={value}
          run={(): void => {
            const msg = new ROSLIB.Message({
              command: value,
            });
            if (gameCommandPub !== null) {
              gameCommandPub.publish(msg);
            }
          }}
          key={value}
        />
      ))}
      <ModalWrapper show={showSelector}>
      {["simon_says", "target_touch"].includes(gameType) && (
        <div>
        <h3>Select Bucket</h3>
        <label htmlFor="selectGameBucket">
          Game Bucket:
          <select
            id="selectGameBucket"
            onChange={(obj): void => {
              const newId: number = parseInt(obj.target.value, 10);
              setGbID(newId);
            }}
          >
            {buckets
              .map((value, idx) => ({ idx, ...value }))
              .filter((value) => value.targeted_game == gameType)
              .map((value, idx) => (
                <option key={idx} value={value.idx}>
                  {value.name}
                </option>
              ))}
          </select>
        </label>
        </div>)
}
        {gameType == "target_touch" && (
          //TODO: insert a number input for number of reps
          <label htmlFor="reps">
            Reps / point:
            <input
              id="reps"
              type="number"
              min="0"
              value={reps}
              onChange={(obj): void => {
                setReps(parseInt(obj.target.value, 10));
              }}
            />
          </label>
        )}
        {gameType == "target_touch" && (
          <label htmlFor="min_steps">
            Min # of steps:
            <input
              id="min_steps"
              type="number"
              min="1"
              value={minSteps}
              onChange={(obj): void => {
                setMinSteps(parseInt(obj.target.value, 10));
              }}
            />
          </label>
        )}
        {gameType == "target_touch" && (
          <label htmlFor="max_steps">
            Max # of steps:
            <input
              id="max_steps"
              type="number"
              min={minSteps}
              value={maxSteps}
              onChange={(obj): void => {
                setMaxSteps(parseInt(obj.target.value, 10));
              }}
            />
          </label>
        )}
        {gameType == "simon_says" && (
          <label htmlFor="bimanual">
            Bimanual (simultaneous)?:
            <button
              id="bimanual"
              type="button"
              onClick={(): void => {
                setBimanual(!bimanual);
              }}
            >
              {bimanual ? "bimanual" : "unimanual"}
            </button>
          </label>
        )}

        <button
          type="button"
          onClick={(): void => {
            if (!connected) {
              console.error("tried to play game when not connected");
              return;
            }
            let steps = [] as StepDef[];
            if(["simon_says", "target_type"].includes(gameType)){
            if (buckets[gbID] === undefined) {
              console.error("tried to play a game with a bad game id: " + gbID);
              return;
            }
             steps = buckets[gbID].steps;
          }
            const gameDef = new ROSLIB.Message({
              game_type: gameType, // eslint-disable-line
              steps: steps,
              reps: reps,
              min_steps: minSteps,
              max_steps: maxSteps,
              bimanual: bimanual,
            });
            if (gameDefPub !== null) {
              gameDefPub.publish(gameDef);
              console.log(`sent command to play ${gameType}`);
            } else {
              console.error("not able to publish game def");
            }
            setShowSelector(false);
          }}
        >
          Start!
        </button>
        <button
          type="button"
          onClick={(): void => {
            setShowSelector(false);
          }}
        >
          Cancel
        </button>
      </ModalWrapper>
    </div>
  );
};

export default GameContainer;
