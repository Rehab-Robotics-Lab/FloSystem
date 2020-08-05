import React from "react";
import ReactDOM from "react-dom";
import "./index.css";
import App from "./App";
import * as serviceWorker from "./serviceWorker";
import { BrowserRouter } from "react-router-dom";

import Honeybadger from "honeybadger-js";
import ErrorBoundary from "@honeybadger-io/react";
import { datadogLogs } from "@datadog/browser-logs";

const honeybadger = Honeybadger.configure({
  apiKey: "3f9e99e4"
});

datadogLogs.init({
  clientToken: "pubd1da6c3a72adcfec0f6fa5d9814aaef5",
  datacenter: "us",
  forwardErrorsToLogs: true,
  sampleRate: 100
});

datadogLogs.logger.log("Page Loaded");
console.log("Page Loaded - console");

ReactDOM.render(
  <ErrorBoundary honeybadger={honeybadger}>
    <BrowserRouter>
      <App />
    </BrowserRouter>
  </ErrorBoundary>,
  document.getElementById("root")
);

// If you want your app to work offline and load faster, you can change
// unregister() to register() below. Note this comes with some pitfalls.
// Learn more about service workers: https://bit.ly/CRA-PWA
serviceWorker.unregister();
