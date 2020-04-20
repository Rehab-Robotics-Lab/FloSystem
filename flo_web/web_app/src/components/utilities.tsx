//https://stackoverflow.com/questions/46140764/polling-api-every-x-seconds-with-react

import React, { useRef, useEffect, useState } from "react";

export const SetInterval = (callback: () => void, delay: number) => {
  const savedCallback = useRef<() => void>(() => {});

  useEffect(() => {
    savedCallback.current = callback;
  }, [callback]);

  useEffect(() => {
    function tick() {
      savedCallback.current();
    }
    tick();
    if (delay !== null) {
      const id = setInterval(tick, delay);
      return () => clearInterval(id);
    }
  }, [delay]);
};
