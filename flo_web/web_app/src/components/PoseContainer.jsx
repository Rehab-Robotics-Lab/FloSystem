import React, { useEffect, useState } from 'react';
import * as ROSLIB from 'roslib';
import PropTypes from 'prop-types';

const poseProp = PropTypes.shape({
  description: PropTypes.string,
  joint_names: PropTypes.array,
  joint_positions: PropTypes.array,
});

function Pose({ pose, addToMoveList }) {
  return (
    <tr>
      <td className="pose">
        {pose.pose.description}
      </td>
      <td>
        <button type="button" onClick={() => { addToMoveList(pose); }}>
Add to move list
        </button>
      </td>
    </tr>
  );
}

Pose.propTypes = {
  pose: PropTypes.shape({
    pose: poseProp,
    id: PropTypes.number,
  }).isRequired,
  addToMoveList: PropTypes.func.isRequired,
};

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
function PoseContainer({ ros, connected }) {
  const [PosesList, setPosesList] = useState([]);
  const [MovesList, setMoveList] = useState([]);

  const addToMoveList = (value) => {
    setMoveList([...MovesList, { time: 2, pose: value, lr: 'right' }]);
  };

  const setTime = (id, time) => {
    const moveListN = [...MovesList];
    const target = moveListN[id];
    target.time = time;
    moveListN[id] = target;
    setMoveList(moveListN);
  };

  const toggleLR = (id) => {
    const moveListN = [...MovesList];
    const target = moveListN[id];
    target.lr = (target.lr === 'left' ? 'right' : 'left');
    moveListN[id] = target;
    setMoveList(moveListN);
  };

  const moveUp = (id) => {
    if (id === 0) return;
    const moveListN = [...MovesList];
    moveListN.splice(id, 0, moveListN.splice(id - 1, 1)[0]);
    setMoveList(moveListN);
  };

  const moveDown = (id) => {
    if (id === MovesList.length - 1) return;
    const moveListN = [...MovesList];
    moveListN.splice(id, 0, moveListN.splice(id + 1, 1)[0]);
    setMoveList(moveListN);
  };

  const remove = (id) => {
    const moveListN = [...MovesList];
    moveListN.splice(id, 1);
    setMoveList(moveListN);
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


  // get all of the poses
  useEffect(() => {
    if (!connected) return;

    const searchPosesClient = new ROSLIB.Service({
      ros,
      name: '/search_pose',
      serviceType: 'flo_core/SearchPose',
    });

    const request = new ROSLIB.ServiceRequest({ search: '' });

    searchPosesClient.callService(request, (resp) => {
      const poses = [];
      for (let i = 0; i < resp.ids.length; i += 1) {
        poses.push({ id: resp.ids[i], pose: resp.poses[i] });
      }
      setPosesList(poses);
    });
  }, [connected]);


  return (
    <div id="movement-container" style={{ display: 'flex' }}>


      <div id="poses-container">
        <h2>Available Poses:</h2>
        <table className="poses-list">
          <tbody>
            {
                PosesList.map((value) => (
                  <Pose
                    id={value.id}
                    pose={value}
                    addToMoveList={addToMoveList}
                  />
                ))
            }
          </tbody>
        </table>
      </div>


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
        <button type="button" onClick={() => { runSequence(); }}>Run Sequence</button>
      </div>


    </div>
  );
}

PoseContainer.defaultProps = {
  ros: null,
};

PoseContainer.propTypes = {
  ros: PropTypes.object, // eslint-disable-line react/forbid-prop-types
  connected: PropTypes.bool.isRequired,
};


export default PoseContainer;
