import React, { useEffect, useReducer, useState } from "react";
import * as ROSLIB from "roslib";
import { SetSpeechTarget, SetSpeaking, Utterance } from "../../robotController";
import SavedSpeech from "./SavedSpeech";
import { basicBlock } from "../../../styleDefs/styles";

const utterancesLength = 3;

enum speechStates {
  UNKNOWN = -1,
  WAITING = 0,
  SYNTHESIZING = 1,
  PLAYING = 2,
  ERROR = 3
}

interface TTSState {
  state: speechStates;
  text: string;
}

interface TTSUtterances {
  text: string;
}

function reducer(state: string[], newVal: string): string[] {
  return [newVal].concat(state).slice(0, utterancesLength - 1);
}

interface SpeechProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  speechTarget: Utterance;
  setSpeechTarget: SetSpeechTarget;
  setSpeaking: SetSpeaking;
  speaking: boolean;
}

// Takes a parameter ros, which is the connection to ros
const Speech: React.FunctionComponent<SpeechProps> = ({
  ros,
  connected,
  speechTarget,
  setSpeechTarget,
  setSpeaking,
  speaking
}) => {
  const [speechState, setSpeechState] = useState<TTSState>({
    state: speechStates.UNKNOWN,
    text: ""
  });
  const [utterances, setUtterances] = useReducer(reducer, []);

  useEffect(() => {
    if (!connected) return;

    const stateListener = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "tts_state",
      messageType: "flo_core_defs/TTSState"
    });
    const slCB = (msg: ROSLIB.Message): void => {
      setSpeechState(msg as TTSState);
    };
    stateListener.subscribe(slCB);
    console.log("subscribed to tts_state");

    const utteranceListener = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "tts_utterances",
      messageType: "flo_core_defs/TTSUtterances"
    });
    const ulCB = (msg: ROSLIB.Message): void => {
      setUtterances((msg as TTSUtterances).text);
    };
    utteranceListener.subscribe(ulCB);
    console.log("Subscribed to tts_utterances");
    return (): void => {
      stateListener.unsubscribe(slCB);
      utteranceListener.unsubscribe(ulCB);
    };
  }, [connected, ros]);

  const runSpeech = (): void => {
    setSpeaking(true);
    const actionClient = new ROSLIB.ActionClient({
      ros: ros as ROSLIB.Ros,
      serverName: "/tts",
      actionName: "tts/SpeechAction",
      timeout: 1 //Not sure about this value here. needs testing
    });
    console.log("connected to speech action server");

    const metadata = JSON.stringify({
      text_type: "ssml", // eslint-disable-line
      voice_id: "Salli" // eslint-disable-line
    });
    const goal = new ROSLIB.Goal({
      actionClient,
      goalMessage: {
        text: "<speak>" + speechTarget.text + "</speak>",
        metadata: metadata
      }
    });

    setSpeechTarget({
      text: speechTarget.text,
      metadata: metadata,
      fileLocation: null
    });

    goal.on("feedback", fb => {
      //TODO: implement voice feedback
      console.log("got feedback on speaking");
    });

    goal.on("result", res => {
      setSpeechTarget({
        text: speechTarget.text,
        metadata: speechTarget.metadata,
        fileLocation: res.response
      });
      setSpeaking(false);
      console.log("done speaking");
      //TODO do something to handle state of the result response
      // should get a useful response
    });

    //TODO put an indication that speaking is underway
    setSpeaking(true);
    goal.send();
    console.log("sent request to speak");
  };

  return (
    <div
      style={Object.assign({}, basicBlock, {
        maxWidth: "200px"
      })}
    >
      <h2>Speech</h2>
      <span>state: {speechStates[speechState.state]}</span>
      <label htmlFor="speechTarget">
        String to speak (SSML):
        <input
          type="text"
          name="speechTarget"
          value={speechTarget.text}
          onChange={(e): void =>
            setSpeechTarget({
              text: e.target.value,
              metadata: null,
              fileLocation: null
            })
          }
        />
      </label>

      <button
        type="button"
        onClick={(): void => {
          runSpeech();
        }}
        disabled={!connected || speaking}
      >
        Speak
      </button>

      <button
        type="button"
        onClick={(): void => {
          setSpeechTarget({ text: "", metadata: null, fileLocation: null });
        }}
        disabled={speechTarget.text === ""}
      >
        Clear
      </button>

      <SavedSpeech
        ros={ros}
        connected={connected}
        speechTarget={speechTarget}
        setSpeechTarget={setSpeechTarget}
        speaking={speaking}
      />
      <div>
        <h3>Recent Utterances</h3>
        {utterances.map(value => (
          <div>{value}</div>
        ))}
      </div>
    </div>
  );
};

export default Speech;
