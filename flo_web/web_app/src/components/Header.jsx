import React, { useState } from 'react';
import * as ROSLIB from 'roslib';

function Header({
  ros, setRos, errorList, addError, connected, setConnected,
}) {
  const [ipAddr, setIpAddr] = useState('localhost');
  const [ipPort, setIpPort] = useState('9090');

  const errorWrapper = () => {
    addError('ROS Connection Error', 'Header');
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (!(ipAddr && ipPort)) return;
    const ros = new ROSLIB.Ros({
      url: `ws://${ipAddr}:${ipPort}`,
    });
    ros.on('error', (err) => { errorWrapper(err); });
    ros.on('connection', () => { setConnected(true); });
    ros.on('close', () => { setConnected(false); });
    setRos(ros);
  };
  const connectedString = () => {
    if (!connected) {
      return <span style={{ color: 'red' }}>Not Connected</span>;
    }
    if (connected) {
      return <span style={{ color: 'green' }}>Connected</span>;
    }
  };

  return (
    <div>
      <form onSubmit={handleSubmit}>
        <label>
      IP Address:
          <input type="text" name="ip_addr" value={ipAddr} onChange={(e) => setIpAddr(e.target.value)} />
        </label>
        <label>
      IP Port:
          <input type="text" name="ip_port" value={ipPort} onChange={(e) => setIpPort(e.target.value)} />
        </label>
        <input type="submit" value="Connect" disabled={connected} />
      </form>
      {connectedString()}
    </div>
  );
}

export default Header;
