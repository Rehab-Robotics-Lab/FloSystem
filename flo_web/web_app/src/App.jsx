import React, { useState } from 'react';
import './App.css';
import Header from './components/Header';
import URDF from './components/urdf';
import PoseContainer from './components/PoseContainer';
import ErrorDisplay from './components/ErrorDisplay';
import SequenceContainer from './components/SequenceContainer';
import colors from './styleDefs/colors';

function App() {
  const [ros, setRos] = useState(null);
  const [errorList, setErrorList] = useState([]);
  const [connected, setConnected] = useState(false);
  const [MovesList, setMovesList] = useState([]);

  const addToMoveList = (value) => {
    setMovesList([...MovesList, { time: 2, pose: value, lr: 'right' }]);
  };

  const setConnectedWrap = (con) => {
    if (con === false && ros !== null) {
      setRos(null);
    }
    setConnected(con);
  };

  const addError = (text, src) => {
    const newError = { text, time: new Date(), src };
    setErrorList([...errorList, newError]);
  };

  return (
    <div className="App">
      <Header
        setRos={setRos}
        addError={addError}
        connected={connected}
        setConnected={setConnectedWrap}
      />
      <div className="body" style={{ backgroundColor: colors.gray.dark2 }}>
        <div className="visualFeeds">
          <URDF ros={ros} connected={connected} />
        </div>
        <PoseContainer
          ros={ros}
          connected={connected}
          addToMoveList={addToMoveList}
        />
        <SequenceContainer
          ros={ros}
          connected={connected}
          MovesList={MovesList}
          setMovesList={setMovesList}
        />
        <ErrorDisplay errorList={errorList} />
      </div>
    </div>
  );
}

export default App;
