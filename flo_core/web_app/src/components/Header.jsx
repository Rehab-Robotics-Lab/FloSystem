import React from 'react';

function Header({ ros, setRos }) {
  return (
    <div>
      <form>
        <label>
      IP Address:
          <input type="text" name="ip_addr" placeholder="xxx.xxx.xxx" />
        </label>
        <label>
      IP Port:
          <input type="text" name="ip_port" />
        </label>
        <input type="submit" value="Connect" />
      </form>
      <span style={{ color: ros === null ? 'red' : 'green' }}>Not Connected</span>
    </div>
  );
}

export default Header;
