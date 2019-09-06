import PropTypes from 'prop-types';

export default null;

// -----      Errors     -----
const errorItem = PropTypes.shape({
  text: PropTypes.string,
  time: PropTypes.instanceOf(Date),
  src: PropTypes.string,
});

export const errorPropDef = {
  item: errorItem,
};

export const errorListPropDef = {
  errorList: PropTypes.arrayOf(errorItem),
};

// -----      Header     -----
export const headerPropDef = {
  setRos: PropTypes.func,
  addError: PropTypes.func,
  connected: PropTypes.bool,
  setConnected: PropTypes.func,
};


// -----      Pose     -----
const InnerPoseProp = PropTypes.shape({
  description: PropTypes.string,
  joint_names: PropTypes.array,
  joint_positions: PropTypes.array,
});

const poseWrapperPropDef = PropTypes.shape({
  pose: InnerPoseProp,
  id: PropTypes.number,
});
export const posePropDef = {
  pose: poseWrapperPropDef.isRequired,
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
export const movePropDef = PropTypes.shape({
  id: PropTypes.number,
  pose: InnerPoseProp,
  time: PropTypes.number,
  lr: PropTypes.string,
  setTime: PropTypes.func,
  toggleLR: PropTypes.func,
  moveUp: PropTypes.func,
  moveDown: PropTypes.func,
  remove: PropTypes.func,
}).isRequired;

export const sequenceRunContainerPropDef = {
  ros: PropTypes.object, // eslint-disable-line react/forbid-prop-types
  connected: PropTypes.bool.isRequired,
  MovesList: PropTypes.arrayOf(PropTypes.shape({
    time: PropTypes.number,
    pose: poseWrapperPropDef,
    lr: PropTypes.string,
    status: PropTypes.string,
  })).isRequired,
  setMovesList: PropTypes.func.isRequired,
};


// -----      URDF     -----
export const urdfPropDef = {
  ros: PropTypes.object, // eslint-disable-line react/forbid-prop-types
  connected: PropTypes.bool.isRequired,
};
