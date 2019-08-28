import React, { useState, useEffect } from 'react';
import './App.css';
import * as ROSLIB from 'roslib';
import Header from './components/Header.jsx';
import URDF from './components/urdf.jsx';

function App() {
  const [ros, setRos] = useState(null);
  const [errorList, setErrorList] = useState([]);
  const [connected, setConnected] = useState(false);

  const setConnectedWrap = (con) => {
    if (con === false && ros !== null) {
      setRos(null);
    }
    setConnected(con);
  };

  const addError = (err) => { setErrorList([...errorList, err]); };

  return (
    <div className="App">
      Flo control center
      <Header ros={ros} setRos={setRos} errorList={errorList} addError={addError} connected={connected} setConnected={setConnectedWrap} />
      <URDF ros={ros} connected={connected} />
    </div>
  );
}

export default App;
