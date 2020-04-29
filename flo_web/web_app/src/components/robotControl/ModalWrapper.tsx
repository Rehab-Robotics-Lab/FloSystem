import React from "react";

interface ModalWrapperProps {
  show: boolean;
}

const ModalWrapper: React.FunctionComponent<ModalWrapperProps> = ({
  show,
  children
}) => {
  if (!show) return null;
  return (
    <div
      style={{
        position: "fixed",
        bottom: 0,
        left: 0,
        right: 0,
        top: 0,
        background: "rgba(0,0,0,.3)",
        display: "flex",
        justifyContent: "center",
        alignItems: "center"
      }}
    >
      <div
        style={{
          width: "95%",
          background: "white",
          borderRadius: "10px",
          maxWidth: "500px",
          position: "relative",
          textAlign: "center",
          display: "flex",
          flexDirection: "column"
        }}
      >
        {children}
      </div>
    </div>
  );
};

export default ModalWrapper;
