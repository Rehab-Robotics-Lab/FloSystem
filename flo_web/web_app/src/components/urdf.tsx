import React, { useEffect, useState } from "react";
import * as ROS3D from "ros3d";
import * as ROSLIB from "roslib";

interface URDFProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
}
// Takes a parameter ros, which is the connection to ros
const URDF: React.FunctionComponent<URDFProps> = ({ ros, connected }) => {
  const [viewer, setViewer] = useState<ROS3D.Viewer | null>(null);
  const [client, setClient] = useState<ROS3D.UrdfClient | null>(null);

  // We need to wait until the target diff exists
  useEffect(() => {
    // if (viewer !== null) return; I don't think
    // this is an issue since
    // this only runs once
    // The view window
    const vw = new ROS3D.Viewer({
      divID: "urdf",
      width: 400,
      height: 400,
      antialias: true,
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
  }, []);

  useEffect(() => {
    if (!connected) {
      if (client && viewer && viewer.scene) {
        viewer.scene.remove(client.urdf);
        setClient(null);
      }
      return;
    }
    if (client !== null) return;

    // The connection to move things around, thresholded to prevent too many redraws
    const tfClient = new ROSLIB.TFClient({
      ros: ros as ROSLIB.Ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0
    });

    // The URDF Loader and drawer
    if (viewer === null || viewer.scene === null) return;
    const clientT = new ROS3D.UrdfClient({
      ros: ros as ROSLIB.Ros,
      tfClient,
      path: `${process.env.PUBLIC_URL}/mesh_root/`,
      rootObject: viewer.scene
      // loader: ROS3D.COLLADA_LOADER2,
    });

    setClient(clientT);
  }, [connected, client, ros, viewer]);

  return <div id="urdf" />;
};

export default URDF;
