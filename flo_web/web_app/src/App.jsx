import React, { useState } from 'react';
import './App.css';
import * as ROSLIB from 'roslib';
import Header from './components/Header.jsx';
import URDF from './components/urdf.jsx';

function App() {
  const [ros, setRos] = useState(null);
  return (
    <div className="App">
      Flo control center
      <Header ros={ros} setRos={setRos} />
      //<URDF ros={ros} />
    </div>
  );
}

export default App;
