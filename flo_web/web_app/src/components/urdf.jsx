import React, { useEffect, useState } from 'react';
import * as ROS3D from 'ros3d';
import * as ROSLIB from 'roslib';

// Takes a parameter ros, which is the connection to ros
function URDF({ ros, connected }) {
  const [viewer, setViewer] = useState(null);

  // We need to wait until the target diff exists
  useEffect(() => {
  // The view window
    const vw = new ROS3D.Viewer({
      divID: 'urdf',
      width: 400,
      height: 200,
      antialias: true,
      background: '#fefef', // sets the background color
      alpha: 0.3, // background transparency
      cameraPose: { x: 0.15, y: 0.5, z: 0.05 },
    });


    // A grid
    vw.addObject(new ROS3D.Grid());

    setViewer(vw);
  }, []);

  useEffect(() => {
    if (!connected) return;
    // The connection to move things around, thresholded to prevent too many redraws
    const tfClient = new ROSLIB.TFClient({
      ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
    });

    // The URDF Loader and drawer
    const client = new ROS3D.UrdfClient({
      ros,
      tfClient,
      path: `${process.env.PUBLIC_URL}/mesh_root/`,
      rootObject: viewer.scene,
      // loader: ROS3D.COLLADA_LOADER_2,
    });
  });

  return (
    <div id="urdf" />
  );
}

export default URDF;
