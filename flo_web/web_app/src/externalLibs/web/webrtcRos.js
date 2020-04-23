const newStreamId = function() {
  return (
    "webrtc_ros-stream-" + Math.floor(Math.random() * 1000000000).toString()
  );
};

const WebrtcRosConnection = function(signalingServerPath, configuration) {
  this.signalingServerPath = signalingServerPath;
  this.onConfigurationNeeded = undefined;
  this.signalingChannel = null;
  this.peerConnection = null;
  this.peerConnectionMediaConstraints = {
    optional: [{ DtlsSrtpKeyAgreement: true }]
  };
  this.peerConnectionConfiguration = configuration;

  this.lastConfigureActionPromise = Promise.resolve([]);

  this.addStreamCallbacks = {};
  this.removeTrackCallbacks = {};
};

WebrtcRosConnection.prototype.connect = function() {
  this.close();
  this.signalingChannel = new WebSocket(this.signalingServerPath);
  this.signalingChannel.onmessage = e => {
    this.onSignalingMessage(e);
  };
  this.signalingChannel.onopen = () => {
    console.log("WebRTC signaling connection established");
    if (this.onConfigurationNeeded) {
      this.onConfigurationNeeded();
    }
  };
  this.signalingChannel.onerror = () => {
    console.error("WebRTC signaling error");
  };
  this.signalingChannel.onclose = () => {
    console.log("WebRTC signaling connection closed");
  };
  this.peerConnection = new RTCPeerConnection(
    this.peerConnectionConfiguration,
    this.peerConnectionMediaConstraints
  );
  this.peerConnection.onicecandidate = event => {
    if (event.candidate) {
      const candidate = {
        sdp_mline_index: event.candidate.sdpMLineIndex,
        sdp_mid: event.candidate.sdpMid,
        candidate: event.candidate.candidate,
        type: "ice_candidate"
      };
      this.signalingChannel.send(JSON.stringify(candidate));
    }
  };
  this.peerConnection.ontrack = event => {
    const callbackData = this.addStreamCallbacks[event.streams[0].id];
    if (callbackData) {
      event.streams[0].onremovetrack = event => {
        const callbackData = this.removeTrackCallbacks[event.track.id];
        if (callbackData) {
          callbackData.resolve({
            track: event.track
          });
        }
      };
      callbackData.resolve({
        stream: event.streams[0],
        remove: new Promise((resolve, reject) => {
          this.removeTrackCallbacks[event.track.id] = {
            resolve: resolve,
            reject: reject
          };
        })
      });
    }
  };
};

WebrtcRosConnection.prototype.close = function() {
  if (this.peerConnection) {
    this.peerConnection.close();
    this.peerConnection = null;
  }
  if (this.signalingChannel) {
    this.signalingChannel.close();
    this.signalingChannel = null;
  }
};

WebrtcRosConnection.prototype.onSignalingMessage = function(e) {
  const dataJson = JSON.parse(e.data);
  if (dataJson.type === "offer") {
    console.log("Received WebRTC offer via WebRTC signaling channel");
    this.peerConnection.setRemoteDescription(
      new RTCSessionDescription(dataJson),
      () => {
        this.sendAnswer();
      },
      function(event) {
        console.error("onRemoteSdpError", event);
      }
    );
  } else if (dataJson.type === "ice_candidate") {
    console.log("Received WebRTC ice_candidate via WebRTC signaling channel");
    const candidate = new RTCIceCandidate({
      sdpMLineIndex: dataJson.sdp_mline_index,
      candidate: dataJson.candidate
    });
    this.peerConnection.addIceCandidate(candidate);
  } else {
    console.warn(
      "Received unknown message type '" +
        dataJson.type +
        "' via WebRTC signaling channel"
    );
  }
};

WebrtcRosConnection.prototype.sendAnswer = function() {
  const mediaConstraints = { optional: [{ OfferToReceiveVideo: true }] };
  this.peerConnection.createAnswer(
    sessionDescription => {
      this.peerConnection.setLocalDescription(sessionDescription);
      const data = JSON.stringify(sessionDescription);
      this.signalingChannel.send(data);
    },
    function(error) {
      console.warn("Create answer error:", error);
    },
    mediaConstraints
  );
};

WebrtcRosConnection.prototype.addRemoteStream = function(config) {
  const stream_id = newStreamId();

  this.lastConfigureActionPromise = this.lastConfigureActionPromise.then(
    actions => {
      actions.push({ type: "add_stream", id: stream_id });
      if (config.video) {
        actions.push({
          type: "add_video_track",
          stream_id: stream_id,
          id: stream_id + "/" + config.video.id,
          src: config.video.src
        });
      }
      if (config.audio) {
        actions.push({
          type: "add_audio_track",
          stream_id: stream_id,
          id: stream_id + "/" + config.audio.id,
          src: config.audio.src
        });
      }
      return actions;
    }
  );
  return new Promise((resolve, reject) => {
    this.addStreamCallbacks[stream_id] = {
      resolve: resolve,
      reject: reject
    };
  });
};

WebrtcRosConnection.prototype.removeRemoteStream = function(stream) {
  this.lastConfigureActionPromise = this.lastConfigureActionPromise.then(
    actions => {
      actions.push({ type: "remove_stream", id: stream.id });
      return actions;
    }
  );
};
WebrtcRosConnection.prototype.addLocalStream = function(
  user_media_config,
  local_stream_config
) {
  return new Promise((resolve, reject) => {
    this.lastConfigureActionPromise = this.lastConfigureActionPromise.then(
      actions => {
        return navigator.mediaDevices
          .getUserMedia(user_media_config)
          .then(stream => {
            actions.push({ type: "expect_stream", id: stream.id });
            if (local_stream_config.video) {
              actions.push({
                type: "expect_video_track",
                stream_id: stream.id,
                id: stream.getVideoTracks()[0].id,
                dest: local_stream_config.video.dest
              });
            }
            this.peerConnection.addStream(stream);
            resolve({
              stream: stream,
              remove: new Promise((resolve, reject) => {
                console.log("*************");
                console.log("this: ");
                console.log(this);
                console.log("stream: ");
                console.log(stream);
                this.removeStreamCallbacks[stream.id] = {
                  resolve: resolve,
                  reject: reject
                };
              })
            });
            return actions;
          });
      }
    );
  });
};

WebrtcRosConnection.prototype.removeLocalStream = function(stream) {
  this.lastConfigureActionPromise = this.lastConfigureActionPromise.then(
    actions => {
      console.log("Removing stream");
      this.peerConnection.removeStream(stream);
      const callbackData = this.removeStreamCallbacks[stream.id];
      if (callbackData) {
        callbackData.resolve({
          stream: stream
        });
      }
      return actions;
    }
  );
};

WebrtcRosConnection.prototype.sendConfigure = function() {
  const currentLastConfigureActionPromise = this.lastConfigureActionPromise;
  this.lastConfigureActionPromise = Promise.resolve([]);

  currentLastConfigureActionPromise.then(actions => {
    const configMessage = { type: "configure", actions: actions };
    this.signalingChannel.send(JSON.stringify(configMessage));
    console.log("WebRTC ROS Configure: ", actions);
  });
};

const WebrtcRos = {
  createConnection: function(signalingServerPath, configuration) {
    return new WebrtcRosConnection(signalingServerPath, configuration);
  }
};

export default WebrtcRos;
