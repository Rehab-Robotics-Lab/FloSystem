import PropTypes from 'prop-types';

export default null;

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

export const headerPropDef = {
  setRos: PropTypes.func.isRequired,
  addError: PropTypes.func.isRequired,
  connected: PropTypes.bool.isRequired,
  setConnected: PropTypes.func.isRequired,
};


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
