import React, { useState } from 'react';
import './App.css';
import Header from './components/Header';
import URDF from './components/urdf';
import PoseContainer from './components/PoseContainer';

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
      <Header
        ros={ros}
        setRos={setRos}
        errorList={errorList}
        addError={addError}
        connected={connected}
        setConnected={setConnectedWrap}
      />
      <URDF ros={ros} connected={connected} />
      <PoseContainer ros={ros} connected={connected} />
    </div>
  );
}

export default App;
