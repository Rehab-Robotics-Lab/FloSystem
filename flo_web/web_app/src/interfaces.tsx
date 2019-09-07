export default null;

// -----      Errors     -----



// -----      Header     -----


// -----      Pose     -----



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
