FROM osrf/ros:melodic-desktop-full

RUN apt-get update -y && apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-pip

RUN pip install 'mutagen==1.43.0' --user

RUN rosdep update

WORKDIR /catkin_ws/src/LilFloSystem
COPY ./flo_core/CMakeLists.txt        ./flo_core/
COPY ./flo_core/package.xml           ./flo_core/
COPY ./flo_core_defs/CMakeLists.txt   ./flo_core_defs/
COPY ./flo_core_defs/package.xml      ./flo_core_defs/

COPY ./flo_face/flo_face/CMakeLists.txt        ./flo_face/flo_face/
COPY ./flo_face/flo_face/package.xml           ./flo_face/flo_face/
COPY ./flo_face/flo_face_defs/CMakeLists.txt   ./flo_face/flo_face_defs/
COPY ./flo_face/flo_face_defs/package.xml      ./flo_face/flo_face_defs/

COPY ./flo_humanoid/CMakeLists.txt        ./flo_humanoid/
COPY ./flo_humanoid/package.xml           ./flo_humanoid/
COPY ./flo_humanoid_defs/CMakeLists.txt   ./flo_humanoid_defs/
COPY ./flo_humanoid_defs/package.xml      ./flo_humanoid_defs/

COPY ./flo_telepresence/CMakeLists.txt        ./flo_telepresence/
COPY ./flo_telepresence/package.xml           ./flo_telepresence/

COPY ./flo_web/CMakeLists.txt        ./flo_web/
COPY ./flo_web/package.xml           ./flo_web/

COPY ./system_monitor/CMakeLists.txt        ./system_monitor/
COPY ./system_monitor/package.xml           ./system_monitor/

WORKDIR /catkin_ws
RUN rosdep install --from-paths src --ignore-src -r -y --skip-keys "realsense2_camera realsense2_description rosbridge_suite rosbridge_server rosbridge_library rosbridge_msgs video_stream_opencv"

WORKDIR /catkin_ws/src
RUN git clone --single-branch --branch develop https://github.com/RobotWebTools/webrtc_ros.git \
    && cd webrtc_ros \
    && git checkout a2a19da \
    && cd webrtc \
    && touch CATKIN_IGNORE

WORKDIR /catkin_ws/src
RUN git clone --single-branch --branch nousub https://github.com/mjsobrep/rosbridge_suite.git

WORKDIR /catkin_ws/src/LilFloSystem
COPY ./ ./

WORKDIR /catkin_ws
RUN bash /ros_entrypoint.sh catkin_make

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
