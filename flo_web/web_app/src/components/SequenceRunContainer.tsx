import React from 'react';
import * as ROSLIB from 'roslib';
import { movePropDef, sequenceRunContainerPropDef } from '../propTypes';


function Move({
  id, pose, lr, time, setTime, toggleLR, moveUp, moveDown, remove, status,
}) {
  const style = {
    display: 'flex',
    flexDirection: 'row',
    flexWrap: 'wrap',
    background: (id % 2) ? '#bcd2e0' : 'white',
    border: 'none',
  };
  if (status === 'complete') {
    style.border = '2px solid green';
  } else if (status === 'moving') {
    style.border = '2px solid blue';
  }

  return (
    <div style={style}>
      <span style={{ width: '100px' }}>
        {pose.pose.description}
      </span>
      <input
        type="number"
        min="0"
        max="10"
        step="any"
        value={time}
        onChange={(e) => setTime(id, parseFloat(e.target.value))}
        style={{ width: '35px' }}
      />
      <button
        type="button"
        onClick={() => { toggleLR(id); }}
      >
        {lr}
      </button>
      <button
        type="button"
        onClick={() => { moveUp(id); }}
      >
&uarr;
      </button>
      <button
        type="button"
        onClick={() => { moveDown(id); }}
      >
&darr;
      </button>
      <button
        type="button"
        onClick={() => { remove(id); }}
      >
&#10007;
      </button>
    </div>
  );
}

Move.propTypes = movePropDef;

// Takes a parameter ros, which is the connection to ros
function SequenceRunContainer({
  ros, connected, MovesList, setMovesList, moving, setMoving,
}) {
  const setTime = (id, time) => {
    const moveListN = [...MovesList];
    const target = moveListN[id];
    target.time = time;
    moveListN[id] = target;
    setMovesList(moveListN);
  };

  const toggleLR = (id) => {
    const moveListN = [...MovesList];
    const target = moveListN[id];
    target.lr = (target.lr === 'left' ? 'right' : 'left');
    moveListN[id] = target;
    setMovesList(moveListN);
  };

  const moveUp = (id) => {
    if (id === 0) return;
    const moveListN = [...MovesList];
    moveListN.splice(id, 0, moveListN.splice(id - 1, 1)[0]);
    setMovesList(moveListN);
  };

  const moveDown = (id) => {
    if (id === MovesList.length - 1) return;
    const moveListN = [...MovesList];
    moveListN.splice(id, 0, moveListN.splice(id + 1, 1)[0]);
    setMovesList(moveListN);
  };

  const remove = (id) => {
    const moveListN = [...MovesList];
    moveListN.splice(id, 1);
    setMovesList(moveListN);
  };

  const runSequence = () => {
    const actionClient = new ROSLIB.ActionClient({
      ros,
      serverName: '/move',
      actionName: 'flo_humanoid/MoveAction',
    });

    let time = 0;

    // Need to iterate through the move list and do:
    // - add limb prefix
    // - calculate the actual target time
    const targets = [];
    MovesList.forEach((move) => {
      time += move.time;
      const names = [...move.pose.pose.joint_names];
      for (let idx = 0; idx < names.length; idx += 1) {
        names[idx] = `${move.lr}_${names[idx]}`;
      }
      const msg = new ROSLIB.Message({
        name: names,
        position: move.pose.pose.joint_positions,
        target_completion_time: time,
      });
      targets.push(msg);
    });

    const goal = new ROSLIB.Goal({
      actionClient,
      goalMessage: {
        targets,
      },
    });

    goal.on('feedback', (fb) => {
      const curMove = fb.move_number;
      const moveListN = [...MovesList];
      for (let idx = 0; idx < curMove; idx += 1) {
        moveListN[idx].status = 'complete';
      }
      moveListN[curMove].status = 'moving';
      setMovesList(moveListN);
    });

    goal.on('result', (res) => {
      setMoving(false);
      const moveListN = [...MovesList];
      for (let idx = 0; idx < moveListN.length; idx += 1) {
        moveListN[idx].status = 'none';
      }
      setMovesList(moveListN);
    });

    setMoving(true);
    const moveListN = [...MovesList];
    for (let idx = 0; idx < moveListN.length; idx += 1) {
      moveListN[idx].status = 'planned';
    }
    setMovesList(moveListN);

    goal.send();
  };


  return (
    <div
      id="moves-container"
      style={{
        backgroundColor: 'white',
        borderRadius: '25px',
        padding: '10px',
        margin: '10px',
      }}
    >
      <h2>List of moves to make:</h2>
      <div style={{ display: 'flex', flexDirection: 'column' }}>
        <div style={{ display: 'flex', flexDirection: 'row' }}>
          <div>Pose</div>
          <div>Time from prior (s)</div>
          <div>Arm</div>
          <div>Up</div>
          <div>Down</div>
          <div>Delete</div>
        </div>
        {
        MovesList.map((value, index) => (
          <Move
            id={index}
            pose={value.pose}
            lr={value.lr}
            toggleLR={toggleLR}
            time={value.time}
            setTime={setTime}
            moveUp={moveUp}
            moveDown={moveDown}
            remove={remove}
            status={value.status}
          />
        ))
}
      </div>
      <hr />
      <button
        type="button"
        disabled={!connected || moving}
        onClick={() => { runSequence(); }}
      >
Run Sequence
      </button>
      <button
        type="button"
        onClick={() => { setMovesList([]); }}
      >
Clear
      </button>
    </div>
  );
}

SequenceRunContainer.defaultProps = {
  ros: null,
};

SequenceRunContainer.propTypes = sequenceRunContainerPropDef;

export default SequenceRunContainer;
