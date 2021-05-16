import React, { useEffect, useState } from "react";
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

interface StoredUtterance {
  idx: number;
  text: string;
}

interface SpeechProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  speechTarget: Utterance;
  setSpeechTarget: SetSpeechTarget;
  setSpeaking: SetSpeaking;
  speaking: boolean;
}

function syllables(word: string): number {
  word = word.toLowerCase();
  if (word === null) {
    return 0;
  }
  if (word.length <= 3) {
    return 1;
  }
  const clean = word
    .replace(/(?:[^laeiouy]es|ed|lle|[^laeiouy]e)$/, "")
    .replace(/^y/, "")
    .match(/[aeiouy]{1,2}/g);
  if (clean) {
    return clean.length;
  }
  return 0;
}

function countSyllables(sentence: string): number {
  let count = 0;
  const words = sentence.split(" ");

  words.map(function(val) {
    count += syllables(val);
    return null;
  });

  return count;
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
  const [voiceVolume, setVoiceVolume] = useState(1);
  const [captions, setCaptions] = useState(false);
  const [lastUtterance, setLastUtterance] = useState("");

  useEffect(() => {
    if (!connected) return;

    const captionsParam = new ROSLIB.Param({
      ros: ros as ROSLIB.Ros,
      name: "/captions"
    });
    captionsParam.get(val => {
      if (val != null) {
        setCaptions(val);
      }
    });

    const voiceVolumeParam = new ROSLIB.Param({
      ros: ros as ROSLIB.Ros,
      name: "/flo_hum_vol"
    });
    voiceVolumeParam.get(val => {
      if (val != null) {
        setVoiceVolume(val);
      }
    });

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
      //setUtterances((msg as TTSUtterances).text);
      setLastUtterance((msg as TTSUtterances).text);
    };
    utteranceListener.subscribe(ulCB);
    console.log("Subscribed to tts_utterances");
    return (): void => {
      stateListener.unsubscribe(slCB);
      utteranceListener.unsubscribe(ulCB);
    };
  }, [connected, ros]);

  const preemptSpeech = (): void => {
    setSpeaking(true);
    const actionClient = new ROSLIB.ActionClient({
      ros: ros as ROSLIB.Ros,
      serverName: "/tts",
      actionName: "tts/SpeechAction",
      timeout: 1500 //Not sure about this value here. needs testing
    });
    console.log("connected to speech action server");
    actionClient.cancel();
  };

  const runSpeech = (): void => {
    setSpeaking(true);
    const actionClient = new ROSLIB.ActionClient({
      ros: ros as ROSLIB.Ros,
      serverName: "/tts",
      actionName: "tts/SpeechAction",
      timeout: 1500 //Not sure about this value here. needs testing
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

    goal.on("feedback", () => {
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

    // The typing is not complete for the actioni client
    //eslint-disable-next-line @typescript-eslint/no-explicit-any
    (actionClient as any).on("timeout", () => {
      goal.cancel();
      console.error("there was an error speaking, please try again");
      setSpeaking(false);
    });

    goal.on("timeout", () => {
      goal.cancel();
      console.error("there was an error speaking, please try again");
      setSpeaking(false);
    });

    //TODO put an indication that speaking is underway
    setSpeaking(true);
    goal.send((countSyllables(speechTarget.text) / 5) * 1000 * 10);
    console.log("sent request to speak");
  };

  return (
    <div
      style={Object.assign({}, basicBlock, {
        maxWidth: "200px"
      })}
    >
      <h2>Speech</h2>
      <label htmlFor="speechVol">
        Volume:
        <input
          type="range"
          min="0"
          max="1"
          step=".1"
          value={voiceVolume}
          onChange={(e): void => {
            const param = new ROSLIB.Param({
              ros: ros as ROSLIB.Ros,
              name: "/flo_hum_vol"
            });
            const val = parseFloat(e.target.value);
            param.set(val);
            setVoiceVolume(val);
          }}
        />
      </label>
      <label htmlFor="captions">
        Captions:
        <button
          id="captions"
          type="button"
          onClick={(): void => {
            const param = new ROSLIB.Param({
              ros: ros as ROSLIB.Ros,
              name: "/captions"
            });
            param.set(!captions);
            setCaptions(!captions);
          }}
        >
          {captions ? "On" : "Off"}
        </button>
      </label>

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
        disabled={!connected}
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

      <button
        type="button"
        onClick={(): void => {
          preemptSpeech();
        }}
        disabled={
          !connected ||
          !(
            speechStates.SYNTHESIZING === speechState.state ||
            speechStates.PLAYING === speechState.state
          )
        }
      >
        Stop Speeking
      </button>

      <SavedSpeech
        ros={ros}
        connected={connected}
        speechTarget={speechTarget}
        setSpeechTarget={setSpeechTarget}
        speaking={speaking}
      />
      <div>
        <h3>Last Utterance</h3>
        {lastUtterance}
        {/*utterances.map((value) => (
          <div key={value.idx}>{value.text}</div>
        ))*/}
      </div>
    </div>
  );
};

export default Speech;
