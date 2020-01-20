import React from "react";

export interface ErrorItem {
  text: string;
  time: Date;
  src: string;
}

interface ErrorProps {
  item: ErrorItem;
}

const Error: React.FunctionComponent<ErrorProps> = ({ item }) => {
  const h = item.time.getHours();
  const m = item.time.getMinutes();
  const s = item.time.getSeconds();
  return (
    <div>
      Error [{item.src}] [{h}:{m}:{s}] : item
      {item.text}
    </div>
  );
};

interface ErrorDisplayProps {
  errorList: Array<ErrorItem>;
}

const ErrorDisplay: React.FunctionComponent<ErrorDisplayProps> = ({
  errorList
}) => {
  return (
    <div
      style={{
        height: "40px",
        overflow: "auto",
        display: "flex",
        flexDirection: "column-reverse",
        color: "red"
      }}
    >
      <div>
        {errorList.map((value, idx) => (
          <Error item={value} key={idx} />
        ))}
      </div>
    </div>
  );
};

export default ErrorDisplay;
