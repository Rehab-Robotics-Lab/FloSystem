import { CSSProperties } from "react";

export const basicBlock: CSSProperties = {
  backgroundColor: "white",
  borderRadius: "25px",
  padding: "10px",
  margin: "10px",
  display: "flex",
  flexDirection: "column",
  overflow: "hidden",
  maxHeight: "400px"
};

export const inputWithSpace: CSSProperties = {
  display: "flex",
  flexDirection: "row",
  justifyContent: "space-between",
  flexWrap: "wrap"
};

export const majorButton: CSSProperties = {
  fontSize: "20px"
};

export const buttonContainer: CSSProperties = {
  display: "flex",
  flexDirection: "row",
  alignItems: "center",
  justifyContent: "space-evenly"
};
