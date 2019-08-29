import React from 'react';
import PropTypes from 'prop-types';

function Error({ item }) {
  const h = item.time.getHours();
  const m = item.time.getMinutes();
  const s = item.time.getSeconds();
  return (
    <div>
      Error [
      {item.src}
] [
      {h}
:
      {m}
:
      {s}
] :
      {' '}
      {item.text}
    </div>
  );
}

Error.propTypes = {
  item: PropTypes.shape({
    text: PropTypes.string,
    time: PropTypes.instanceOf(Date),
    src: PropTypes.string,
  }).isRequired,
};

function ErrorDisplay({ errorList }) {
  return (
    <div style={{
      height: '40px', overflow: 'auto', display: 'flex', flexDirection: 'column-reverse',
    }}
    >
      <div>
        {
        errorList.map((value) => (
          <Error item={value} />
        ))
            }
      </div>
    </div>
  );
}

ErrorDisplay.propTypes = {
  errorList: PropTypes.arrayOf(Error.propTypes).isRequired,
};

export default ErrorDisplay;
