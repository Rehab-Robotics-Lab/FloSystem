import React from "react";
import * as ROSLIB from "roslib";
import { SetSpeechTarget, SetSpeaking, Utterance } from "../App";
import SavedSpeech from "./SavedSpeech";
import { basicBlock } from "../styleDefs/styles";

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
  const runSpeech = (): void => {
    setSpeaking(true);
    const actionClient = new ROSLIB.ActionClient({
      ros: ros as ROSLIB.Ros,
      serverName: "/tts",
      actionName: "tts/SpeechAction",
      timeout: 1 //Not sure about this value here. needs testing
    });

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
    });

    goal.on("result", res => {
      setSpeechTarget({
        text: speechTarget.text,
        metadata: speechTarget.metadata,
        fileLocation: res.response
      });
      setSpeaking(false);
      //TODO do something to handle state of the result response
      // should get a useful response
    });

    //TODO put an indication that speaking is underway
    setSpeaking(true);
    goal.send();
  };

  return (
    <div
      style={Object.assign({}, basicBlock, {
        maxWidth: "200px"
      })}
    >
      <h2>Speech</h2>
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
    </div>
  );
};

export default Speech;
