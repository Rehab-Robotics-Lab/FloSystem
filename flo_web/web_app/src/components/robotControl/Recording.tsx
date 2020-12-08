import React, { useState, useEffect } from "react";
import { majorButton } from "../../styleDefs/styles";
import * as ROSLIB from "roslib";

interface RecordProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}

interface RecordingStatusMsg {
  data: boolean;
}

// Takes a parameter ros, which is the connection to ros
const Record: React.FunctionComponent<RecordProps> = ({ ros, connected }) => {
  const [recordingState, setRecordingState] = useState<boolean>();

  useEffect(() => {
    if (!connected || ros === null) {
      return;
    }
    const topic = new ROSLIB.Topic({
      ros: ros,
      name: "/record_video_status",
      messageType: "std_msgs/Bool"
    });

    const newRecordingStatus = (msg: ROSLIB.Message): void => {
      const cmsg = msg as RecordingStatusMsg;
      setRecordingState(cmsg.data);
    };

    topic.subscribe(newRecordingStatus);

    return (): void => {
      topic.unsubscribe(newRecordingStatus);
    };
  }, [connected, ros]);

  const setRecord = (on: boolean): void => {
    if (!connected || ros === null) {
      return;
    }

    const setRecordingClient = new ROSLIB.Service({
      ros: ros,
      name: "/set_recording",
      serviceType: "flo_core_defs/SetRecording.srv"
    });

    const req = new ROSLIB.ServiceRequest({ record: on });

    setRecordingClient.callService(req, res => {
      console.log("set recording service resp: " + res.success);
    });
  };

  let button;
  let recordingString;
  let recordingColor;
  if (recordingState) {
    button = (
      <button
        type="button"
        disabled={!connected}
        style={majorButton}
        onClick={(): void => setRecord(false)}
      >
        Stop Recording
      </button>
    );
    recordingString = "Recording!";
    recordingColor = { color: "green" };
  } else {
    button = (
      <button
        type="button"
        disabled={!connected}
        style={Object.assign({}, majorButton, { width: "180px" })}
        onClick={(): void => setRecord(true)}
      >
        Start Recording
      </button>
    );
    recordingString = "Not Recording!";
    recordingColor = { color: "red" };
  }
  const recordingSpan = (
    <span style={Object.assign({}, { width: "160px" }, recordingColor)}>
      {recordingString}
    </span>
  );

  return (
    <div
      style={{
        display: "flex",
        flexDirection: "row",
        justifyContent: "center",
        alignItems: "center"
      }}
    >
      {recordingSpan}
      <div
        style={Object.assign({}, majorButton, {
          width: "180px"
        })}
      >
        {button}
      </div>
    </div>
  );
};

export default Record;
