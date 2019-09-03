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
        <button type="button" onClick={() => { addToMoveList(pose); }}>
        {pose.pose.description}
        </button>
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
  }, [connected,ros]);


  return (
    <div id="poses-container" style={{maxWidth: '150px',backgroundColor:'white',borderRadius:'25px',padding:'10px',margin:'10px'}}>
      <h2>Poses:</h2>
      <div style={{display:'flex',flexDirection:'column',overflow:'auto',maxHeight:'400px'}}>
          {
                PosesList.map((value) => (
                  <Pose
                    id={value.id}
                    pose={value}
                    addToMoveList={addToMoveList}
                  />
                ))
            }
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
  addToMoveList: PropTypes.func.isRequired,
};


export default PoseContainer;
