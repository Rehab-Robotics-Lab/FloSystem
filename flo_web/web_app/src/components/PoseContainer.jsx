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


// Takes a parameter ros, which is the connection to ros
function PoseContainer({ ros, connected, addToMoveList }) {
  const [PosesList, setPosesList] = useState([]);

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
  );
}

PoseContainer.defaultProps = {
  ros: null,
};

PoseContainer.propTypes = {
  ros: PropTypes.object, // eslint-disable-line react/forbid-prop-types
  connected: PropTypes.bool.isRequired,
  addToMoveList: PropTypes.func.isRequired,
};


export default PoseContainer;
