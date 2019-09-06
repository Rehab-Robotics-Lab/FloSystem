import React from 'react';
import { errorPropDef, errorListPropDef } from '../propTypes';

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

Error.propTypes = errorPropDef;

function ErrorDisplay({ errorList }) {
  return (
    <div style={{
      height: '40px', overflow: 'auto', display: 'flex', flexDirection: 'column-reverse', color: 'red',
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

ErrorDisplay.propTypes = errorListPropDef;

export default ErrorDisplay;
