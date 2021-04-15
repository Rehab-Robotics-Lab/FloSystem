FROM osrf/ros:melodic-desktop-full

ARG ROS_USER
ENV ROS_USER="${ROS_USER}"

ARG R_GID
ENV R_GID="${R_GID}"
ARG R_UID
ENV R_UID="${R_UID}"

RUN echo "$ROS_USER"

RUN useradd -m $ROS_USER && \
        echo "$ROS_USER:$ROS_USER" | chpasswd && \
        usermod --shell /bin/bash $ROS_USER && \
        usermod -aG sudo $ROS_USER && \
        echo "$ROS_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$ROS_USER && \
        chmod 0440 /etc/sudoers.d/$ROS_USER && \
        # Replace 1000 with your user/group id
        usermod  --uid $R_UID $ROS_USER && \
        groupmod --gid $R_GID $ROS_USER

COPY pulse-client.conf /etc/pulse/client.conf

RUN apt-get update -y && apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-pip python3-pip unzip libpulse0 pulseaudio pulseaudio-utils libcanberra-gtk-module

RUN mkdir /home/$ROS_USER/flo_data \
    && mkdir /home/$ROS_USER/catkin_ws/src -p
RUN chown -R $ROS_USER /home/$ROS_USER

USER $ROS_USER

RUN HOME="/home/$ROS_USER"\
    && export HOME

RUN /ros_entrypoint.sh pip2 install 'mutagen==1.43.0' --user
RUN pip3 install -U boto3

USER root
WORKDIR /aws-temp
RUN curl "https://s3.amazonaws.com/aws-cli/awscli-bundle.zip" -o "awscli-bundle.zip" \
    && unzip awscli-bundle.zip
RUN ./awscli-bundle/install -i /usr/local/aws -b /usr/local/bin/aws \
WORKDIR /aws-temp
RUN rm awscli-bundle.zip && rm -r awscli-bundle

USER $ROS_USER

RUN /ros_entrypoint.sh rosdep update

WORKDIR /home/$ROS_USER/catkin_ws/src/LilFloSystem
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


WORKDIR /home/$ROS_USER/catkin_ws/src

RUN git clone --single-branch --branch develop https://github.com/RobotWebTools/webrtc_ros.git \
    && cd webrtc_ros \
    && git checkout a2a19da \
    && cd webrtc \
    && touch CATKIN_IGNORE

WORKDIR /home/$ROS_USER/catkin_ws/src
RUN git clone --single-branch --branch nousub https://github.com/mjsobrep/rosbridge_suite.git

WORKDIR /home/$ROS_USER/catkin_ws
RUN /ros_entrypoint.sh rosdep install --from-paths src --ignore-src -r -y --skip-keys "realsense2_camera realsense2_description rosbridge_suite rosbridge_server rosbridge_library rosbridge_msgs video_stream_opencv"


WORKDIR /home/$ROS_USER/catkin_ws/src/LilFloSystem
COPY ./ ./

WORKDIR /home/$ROS_USER/catkin_ws
RUN /ros_entrypoint.sh catkin_make
RUN /ros_entrypoint.sh catkin_make install

#RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

USER root
RUN head -n -1 /ros_entrypoint.sh > /tmp.sh \
    && mv /tmp.sh /ros_entrypoint.sh \
    && echo 'source "/home/$ROS_USER/catkin_ws/devel/setup.bash"' >> /ros_entrypoint.sh \
    && echo 'exec $@' >> /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

USER $ROS_USER

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
