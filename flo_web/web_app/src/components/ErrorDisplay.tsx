import React from 'react';

export interface errorItem {
  text: string,
  time: Date,
  src: string,
};

interface errorProps{
    item:errorItem
}

const Error: React.FunctionComponent<errorProps> =({ item }) => {
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
      {' '}item
      {item.text}
    </div>
  );
}

interface errorDisplayProps {
    errorList: Array<errorItem>
}

const  ErrorDisplay:React.FunctionComponent<errorDisplayProps> = ({ errorList }) => {
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

export default ErrorDisplay;
