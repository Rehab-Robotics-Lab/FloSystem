import React, { useState } from 'react';
import * as ROSLIB from 'roslib';
import colors from '../styleDefs/colors';
import { headerPropDef } from '../propTypes';

function Header({
  setRos, addError, connected, setConnected,
}) {
  const [ipAddr, setIpAddr] = useState(window.location.hostname);
  const [ipPort, setIpPort] = useState('9090');

  const errorWrapper = () => {
    addError('ROS Connection Error', 'Header');
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (!(ipAddr && ipPort)) return;
    const newRosConnection = new ROSLIB.Ros({
      url: `ws://${ipAddr}:${ipPort}`,
    });
    newRosConnection.on('error', (err) => { errorWrapper(err); });
    newRosConnection.on('connection', () => { setConnected(true); });
    newRosConnection.on('close', () => { setConnected(false); });
    setRos(newRosConnection);
  };
  const connectedString = () => {
    let toReturn;
    if (!connected) {
      toReturn = <span style={{ color: 'red' }}>Not Connected</span>;
    }
    if (connected) {
      toReturn = <span style={{ color: 'green' }}>Connected</span>;
    }
    return toReturn;
  };

  return (
    <div style={{
      display: 'flex', alignItems: 'stretch', justifyContent: 'space-between', backgroundColor: colors.blue.neutral, color: colors.white,
    }}
    >
      <h1 style={{ margin: '0px' }}>
        Flo Control Center
      </h1>
      <div>
        <form onSubmit={handleSubmit}>
          <label htmlFor="ip_addr">
      IP Address:
            <input type="text" name="ip_addr" value={ipAddr} onChange={(e) => setIpAddr(e.target.value)} />
          </label>
          <label htmlFor="ip_port">
      IP Port:
            <input type="text" name="ip_port" value={ipPort} onChange={(e) => setIpPort(e.target.value)} />
          </label>
          <input type="submit" value="Connect" disabled={connected} />
        </form>
        <b>
          {connectedString()}
        </b>
      </div>
    </div>
  );
}

Header.propTypes = headerPropDef;

export default Header;
