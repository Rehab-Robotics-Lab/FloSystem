import React, { useEffect, useReducer } from "react";
import * as ROSLIB from "roslib";

//https://gist.github.com/mlocati/7210513
function perc2color(perc: number): string {
  perc = 100 - perc;
  let r;
  let g;
  const b = 0;
  if (perc < 50) {
    r = 255;
    g = Math.round(5.1 * perc);
  } else {
    g = 255;
    r = Math.round(510 - 5.1 * perc);
  }
  const h = r * 0x10000 + g * 0x100 + b * 0x1;
  return "#" + ("000000" + h.toString(16)).slice(-6);
}

const statsLength = 20;
const gaugeF = (n: number): string => {
  return Math.round(n).toString();
};

const gaugeFdb = (n: number): string => {
  return Math.round(n).toString() + " dBm";
};

interface GaugeProps {
  value: number;
  label: string;
  valueFormatter?: (arg: number) => string;
  min?: number;
  max?: number;
  invert?: boolean;
}

const Gauge: React.FunctionComponent<GaugeProps> = ({
  value,
  label,
  valueFormatter,
  min,
  max,
  invert,
}) => {
  let valueStr = Math.round(value).toString();
  if (valueFormatter) {
    valueStr = valueFormatter(value);
  }
  let scaleVal = value;
  if (min && max) {
    scaleVal = (100 * (value - min)) / (max - min);
  }
  if (invert) {
    scaleVal = 100 - scaleVal;
  }
  return (
    <div
      style={{
        backgroundColor: perc2color(scaleVal),
        width: "100%",
        height: "auto",
        fontSize: "1vh",
      }}
    >
      {label}:{valueStr}
    </div>
  );
};

Gauge.defaultProps = {
  valueFormatter: (arg: number): string => {
    return Math.round(arg).toString();
  },
  min: 0,
  max: 100,
  invert: false,
};

interface CPUutilMsg {
  percent_utilization: number;
}
interface HDDutilMsg {
  percent_free: number;
}
interface MEMutilMsg {
  percent_used: number;
}
interface NETstatsMsg {
  link_quality: number;
  signal_strength: number;
}

interface SystemMonitorProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}

function reducer(state: number[], newVal: number): number[] {
  return [newVal].concat(state).slice(0, statsLength - 1);
}

const SystemMonitor: React.FunctionComponent<SystemMonitorProps> = ({
  ros,
  connected,
}) => {
  const [cpu, setCpu] = useReducer(reducer, []);
  const [mem, setMem] = useReducer(reducer, []);
  const [hdd, setHdd] = useReducer(reducer, []);
  const [netQ, setNetQ] = useReducer(reducer, []);
  const [netS, setNetS] = useReducer(reducer, []);

  useEffect(() => {
    if (!connected) return;

    const cpuListener = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "cpu_stats",
      messageType: "system_monitor/CPUutil",
    });
    const cpuCB = (msg: ROSLIB.Message): void => {
      setCpu((msg as CPUutilMsg).percent_utilization);
    };
    cpuListener.subscribe(cpuCB);

    const memListener = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "mem_stats",
      messageType: "system_monitor/MEMutil",
    });
    const memCB = (msg: ROSLIB.Message): void => {
      setMem((msg as MEMutilMsg).percent_used);
    };
    memListener.subscribe(memCB);
    console.log("subscribed to memory utilization topic");

    const hddListener = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "hdd_stats",
      messageType: "system_monitor/HDDutil",
    });
    const hddCB = (msg: ROSLIB.Message): void => {
      setHdd((msg as HDDutilMsg).percent_free);
    };
    hddListener.subscribe(hddCB);
    console.log("subscribed to hard drive stats topic");

    const netListener = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: "net_stats",
      messageType: "system_monitor/NETstats",
    });
    const netCB = (msg: ROSLIB.Message): void => {
      setNetQ((msg as NETstatsMsg).link_quality);
      setNetS((msg as NETstatsMsg).signal_strength);
    };
    netListener.subscribe(netCB);
    console.log("subscribed to network stats topic");

    return (): void => {
      cpuListener.unsubscribe(cpuCB);
      memListener.unsubscribe(memCB);
      hddListener.unsubscribe(hddCB);
      netListener.unsubscribe(netCB);
    };
  }, [connected, ros]);

  return (
    <>
      <Gauge value={cpu[0]} label="CPU" valueFormatter={gaugeF} />
      <Gauge value={mem[0]} label="Mem" valueFormatter={gaugeF} />
      <Gauge value={100 - hdd[0]} label="HD" valueFormatter={gaugeF} />
      <Gauge
        value={netQ[0]}
        label="WiFi Quality"
        valueFormatter={gaugeF}
        invert={true}
      />
      <Gauge
        value={netS[0]}
        label="WiFi Strength"
        valueFormatter={gaugeFdb}
        min={-70} //https://codeyarns.com/2017/07/23/dbm-wireless-signal-strength/
        max={-20}
        invert={true}
      />
    </>
  );
};

export default SystemMonitor;
