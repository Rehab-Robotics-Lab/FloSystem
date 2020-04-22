//https://stackoverflow.com/questions/46140764/polling-api-every-x-seconds-with-react

import { useRef, useEffect } from "react";

const SetInterval = (callback: () => void, delay: number): void => {
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

export { SetInterval };
