import PropTypes from 'prop-types';

export default null;

// -----      Errors     -----
const errorItem = PropTypes.shape({
  text: PropTypes.string.isRequired,
  time: PropTypes.instanceOf(Date).isRequired,
  src: PropTypes.string.isRequired,
});

export const errorPropDef = {
  item: errorItem,
};

export const errorListPropDef = {
  errorList: PropTypes.arrayOf(errorItem).isRequired,
};

// -----      Header     -----
export const headerPropDef = {
  setRos: PropTypes.func.isRequired,
  addError: PropTypes.func.isRequired,
  connected: PropTypes.bool.isRequired,
  setConnected: PropTypes.func.isRequired,
};


// -----      Pose     -----
const InnerPoseProp = PropTypes.shape({
  description: PropTypes.string,
  joint_names: PropTypes.array,
  joint_positions: PropTypes.array,
});

export const posePropDef = {
  pose: PropTypes.shape({
    pose: InnerPoseProp,
    id: PropTypes.number,
  }).isRequired,
  addToMoveList: PropTypes.func.isRequired,
};

export const poseContainerPropDef = {
  ros: PropTypes.object, // eslint-disable-line react/forbid-prop-types
  connected: PropTypes.bool.isRequired,
  addToMoveList: PropTypes.func.isRequired,
};


// -----      Sequence     -----
export const sequencePropDef = {
  sequence: PropTypes.shape({
    pose_ids: PropTypes.arrayOf(PropTypes.number),
    times: PropTypes.arrayOf(PropTypes.number),
    arms: PropTypes.arrayOf(PropTypes.string),
    description: PropTypes.string,
    total_time: PropTypes.number,
  }).isRequired,
  setMovesList: PropTypes.func.isRequired,
};

export const sequenceContainerPropDef = {
  ros: PropTypes.object, // eslint-disable-line react/forbid-prop-types
  connected: PropTypes.bool.isRequired,
  addToMoveList: PropTypes.func.isRequired,
};


// -----      Movement     -----
export const movePropDef = {
  id: PropTypes.number.isRequired,
  pose: InnerPoseProp.isRequired,
  time: PropTypes.number.isRequired,
  lr: PropTypes.string.isRequired,
  setTime: PropTypes.func.isRequired,
  toggleLR: PropTypes.func.isRequired,
  moveUp: PropTypes.func.isRequired,
  moveDown: PropTypes.func.isRequired,
  remove: PropTypes.func.isRequired,
};

export const sequenceRunContainerPropDef = {
  ros: PropTypes.object, // eslint-disable-line react/forbid-prop-types
  connected: PropTypes.bool.isRequired,
  MovesList: PropTypes.arrayOf(movePropDef).isRequired,
  setMovesList: PropTypes.func.isRequired,
};


// -----      URDF     -----
export const urdfPropDef = {
  ros: PropTypes.object, // eslint-disable-line react/forbid-prop-types
  connected: PropTypes.bool.isRequired,
};
