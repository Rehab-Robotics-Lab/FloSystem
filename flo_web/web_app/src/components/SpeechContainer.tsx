import React, { useEffect, useState } from "react";
import * as ROSLIB from "roslib";
import { SetSpeechString, SetSpeaking } from "../App";

interface SpeechProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  speechString: string;
  setSpeechString: SetSpeechString;
  setSpeaking: SetSpeaking;
  speaking: boolean;
}
// Takes a parameter ros, which is the connection to ros
const Speech: React.FunctionComponent<SpeechProps> = ({
  ros,
  connected,
  speechString,
  setSpeechString,
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

    const goal = new ROSLIB.Goal({
      actionClient,
      goalMessage: {
        text: speechString,
        metadata: ""
      }
    });

    goal.on("feedback", fb => {
      //TODO: implement voice feedback
    });

    goal.on("result", res => {
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
      style={{
        maxWidth: "300px",
        backgroundColor: "white",
        borderRadius: "25px",
        padding: "10px",
        margin: "10px"
      }}
    >
      <label htmlFor="speechString">
        String to speak:
        <input
          type="text"
          name="speechString"
          value={speechString}
          onChange={(e): void => setSpeechString(e.target.value)}
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
          setSpeechString("");
        }}
        disabled={speechString === ""}
      >
        Clear
      </button>
    </div>
  );
};

export default Speech;
