import React from "react";
import ReactDOM from "react-dom";
import "./index.css";
import App from "./App";
import * as serviceWorker from "./serviceWorker";
import { BrowserRouter } from "react-router-dom";

import { datadogLogs } from "@datadog/browser-logs";

datadogLogs.init({
  clientToken: "pubd1da6c3a72adcfec0f6fa5d9814aaef5",
  datacenter: "us",
  forwardErrorsToLogs: true,
  sampleRate: 100,
});

datadogLogs.logger.log("Page Loaded");
console.log("Page Loaded - console");

ReactDOM.render(
  <BrowserRouter>
    <App />
  </BrowserRouter>,
  document.getElementById("root")
);

// If you want your app to work offline and load faster, you can change
// unregister() to register() below. Note this comes with some pitfalls.
// Learn more about service workers: https://bit.ly/CRA-PWA
serviceWorker.unregister();
