import React, {useState} from 'react';
import * as ROSLIB from 'roslib';

function Header({ ros, setRos }) {
  const [ipAddr, setIpAddr] = useState('localhost');
  const [ipPort, setIpPort] = useState('9090');

    const handleSubmit = (e) => {
        e.preventDefault();
        if (!(ipAddr && ipPort)) return;
        let ros = new ROSLIB.Ros({
            url: 'ws://'+ipAddr+':'+ipPort
        })
        setRos(ros);

    }

  return (
    <div>
      <form onSubmit={handleSubmit}>
        <label>
      IP Address:
          <input type="text" name="ip_addr" value={ipAddr} onChange={(e)=>setIpAddr(e.target.value)}/>
        </label>
        <label>
      IP Port:
          <input type="text" name="ip_port" value={ipPort} onChange={(e)=>setIpPort(e.target.value)}/>
        </label>
        <input type="submit" value="Connect" />
      </form>
      <span style={{ color: ros === null ? 'red' : 'green' }}>Not Connected</span>
    </div>
  );
}

export default Header;
