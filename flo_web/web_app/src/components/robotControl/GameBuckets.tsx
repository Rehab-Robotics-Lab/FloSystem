import React, { useState, useEffect } from "react";
import * as ROSLIB from "roslib";
import { basicBlock } from "../../styleDefs/styles";

interface GameBucketsProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}
// Takes a parameter ros, which is the connection to ros
const GameBuckets: React.FunctionComponent<GameBucketsProps> = ({
  ros,
  connected
}) => {
  return (
    <div style={basicBlock}>
      <h2>GameBuckets</h2>

      <button type="button">Load Bucket</button>
      <button type="button">Save Bucket</button>
      <hr />
    </div>
  );
};

export default GameBuckets;
