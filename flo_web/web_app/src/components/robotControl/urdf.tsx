import React, { useEffect, useState } from "react";
import * as ROS3D from "ros3d";
import * as ROSLIB from "roslib";
import { useRouteMatch } from "react-router-dom";

interface URDFProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}
// Takes a parameter ros, which is the connection to ros
const URDF: React.FunctionComponent<URDFProps> = ({ ros, connected }) => {
  const [viewer, setViewer] = useState<ROS3D.Viewer | undefined>(undefined);
  const divRef = React.useRef<HTMLHeadingElement>(null);

  const { url } = useRouteMatch();
  // We need to wait until the target diff exists
  useEffect(() => {
    // if (viewer !== null) return; I don't think
    // this is an issue since
    // this only runs once
    // The view window
    const vw = new ROS3D.Viewer({
      divID: "urdf",
      width: 300,
      height: 250,
      antialias: false,
      background: "#f2f2f2", // sets the background color
      alpha: 1, // background transparency
      cameraPose: { x: 0.2, y: 0.75, z: 0.05 },
      intensity: 100 // lighting intensity
    });
    // A grid
    // vw.addObject(new ROS3D.Grid({ num_cells: 5, cellSize: 0.1 }));
    vw.directionalLight.position.y = 10;
    // vw.directionalLight.castShaddow = true;
    setViewer(vw);
    console.log("setup urdf viewer");

    return (): void => {
      console.log("stopping urdf viewer");
      vw.stop();
    };
  }, [url]);

  useEffect(() => {
    if (!connected) {
      return;
    }
    // The connection to move things around, thresholded to prevent too many redraws
    const tfClient = new ROSLIB.TFClient({
      ros: ros as ROSLIB.Ros,
      angularThres: 0.1,
      transThres: 0.1,
      rate: 5.0
    });
    console.log("created a new TF client");

    //eslint-disable-next-line no-undef
    console.log(`hosting urdf from : ${process.env.PUBLIC_URL}/mesh_root/`);
    // The URDF Loader and drawer
    if (viewer === undefined || viewer.scene === null) return;
    const clientT = new ROS3D.UrdfClient({
      ros: ros as ROSLIB.Ros,
      tfClient,
      //eslint-disable-next-line no-undef
      path: `${process.env.PUBLIC_URL}/mesh_root/`,
      rootObject: viewer.scene
      // loader: ROS3D.COLLADA_LOADER2,
    });
    console.log("created a new URDF viewer");

    return (): void => {
      if (clientT && viewer && viewer.scene) {
        viewer.scene.remove(clientT.urdf);
        console.log("removed urdf viewer");
      }
      console.log("destroying tf client");
      tfClient.dispose();
    };
  }, [connected, ros, viewer]);

  if (viewer !== undefined && divRef !== null && divRef.current !== null) {
    viewer.resize(divRef.current.offsetWidth, divRef.current.offsetHeight);
  }

  return (
    <div
      style={{
        width: "15%",
        minWidth: "100px"
      }}
    >
      <div
        style={{
          width: "100%",
          paddingTop: "99%",
          height: "0",
          overflow: "hidden",
          position: "relative"
        }}
      >
        <div
          id="urdf"
          ref={divRef}
          style={{
            position: "absolute",
            top: "0",
            left: "0",
            width: "100%",
            height: "100%",
            textAlign: "center"
          }}
        />
      </div>
    </div>
  );
};

export default URDF;
