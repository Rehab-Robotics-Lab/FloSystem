import { CSSProperties } from "react";

export const basicBlock: CSSProperties = {
  backgroundColor: "white",
  borderRadius: "15px",
  padding: "5px",
  margin: "2.5px",
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

export const wrapStyle: CSSProperties = {
  display: "flex",
  flexDirection: "column"
};
