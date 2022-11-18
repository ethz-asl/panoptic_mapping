FROM ros:noetic
ENV TZ=Europe/Zurich
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
    && echo $TZ > /etc/timezone
ENV DEBIAN_FRONTEND="noninteractive"
RUN apt update \
    && apt install -y python3-catkin-tools git
RUN mkdir -p /catkin_ws/src;
WORKDIR /catkin_ws
RUN catkin init
RUN catkin config --extend "/opt/ros/noetic"
RUN catkin config --merge-devel
RUN catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
COPY . /catkin_ws/src/panoptic_mapping
WORKDIR /catkin_ws/src
RUN wstool init . ./panoptic_mapping/panoptic_mapping_https.rosinstall
RUN wstool update
RUN rosdep update
RUN apt update \
    && rosdep install --from-paths . --ignore-src -y --rosdistro noetic
RUN catkin build -j$(nproc) -l$(nproc) panoptic_mapping_utils
