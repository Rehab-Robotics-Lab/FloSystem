import React from 'react';
import * as ROSLIB from 'roslib';
import PropTypes from 'prop-types';

const poseProp = PropTypes.shape({
  description: PropTypes.string,
  joint_names: PropTypes.array,
  joint_positions: PropTypes.array,
});


function Move({
  id, pose, lr, time, setTime, toggleLR, moveUp, moveDown, remove,
}) {
  return (
    <tr>
      <td>
        {pose.pose.description}
      </td>
      <td>
        <input type="number" min="0" max="10" step="any" value={time} onChange={(e) => setTime(id, parseFloat(e.target.value))} />
      </td>
      <td>
        <button type="button" onClick={() => { toggleLR(id); }}>{lr}</button>
      </td>
      <td>
        <button type="button" onClick={() => { moveUp(id); }}>&uarr;</button>
      </td>
      <td>
        <button type="button" onClick={() => { moveDown(id); }}>&darr;</button>
      </td>
      <td>
        <button type="button" onClick={() => { remove(id); }}>&#10007;</button>
      </td>
    </tr>
  );
}

Move.propTypes = {
  id: PropTypes.number.isRequired,
  pose: poseProp.isRequired,
  time: PropTypes.number.isRequired,
  lr: PropTypes.string.isRequired,
  setTime: PropTypes.func.isRequired,
  toggleLR: PropTypes.func.isRequired,
  moveUp: PropTypes.func.isRequired,
  moveDown: PropTypes.func.isRequired,
  remove: PropTypes.func.isRequired,
};

// Takes a parameter ros, which is the connection to ros
function SequenceContainer({
  ros, connected, MovesList, setMovesList,
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
    const targetPub = new ROSLIB.Topic({
      ros,
      name: '/target_joint_states',
      messageType: 'flo_humanoid/JointTarget',
    });

    let time = 0;

    // Need to iterate through the move list and do:
    // - add limb prefix
    // - calculate the actual target time
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
      targetPub.publish(msg);
    });

    const commandPub = new ROSLIB.Topic({
      ros,
      name: '/motor_commands',
      messageType: 'std_msgs/String',
    });

    commandPub.publish({ data: 'move' });
  };


  return (
    <div id="moves-container">
      <h2>List of moves to make:</h2>
      <table className="poses-list">
        <tbody>
          <tr>
            <th>Pose</th>
            <th>Time from prior (s)</th>
          </tr>
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
          />
        ))
}
        </tbody>
      </table>
      <button type="button" disabled={!connected} onClick={() => { runSequence(); }}>Run Sequence</button>
    </div>
  );
}

SequenceContainer.defaultProps = {
  ros: null,
};

SequenceContainer.propTypes = {
  ros: PropTypes.object, // eslint-disable-line react/forbid-prop-types
  connected: PropTypes.bool.isRequired,
  MovesList: PropTypes.arrayOf(Move.propTypes).isRequired,
  setMovesList: PropTypes.func.isRequired,
};


export default SequenceContainer;
