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
