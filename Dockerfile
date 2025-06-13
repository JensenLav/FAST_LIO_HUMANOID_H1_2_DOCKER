FROM osrf/ros:humble-desktop-full
# 0) Remove any old/expired ROS 2 apt entry

# Install dependencies
RUN apt-get update && apt-get install -y \
    libpcl-dev \
    iproute2 \
    libapr1-dev \
    python3-ament-package \
    python3-colcon-common-extensions \
    python3-pip \
    libeigen3-dev \
    libboost-all-dev \
    libaprutil1-dev \
    git \
    cmake \
    iputils-ping \
    build-essential \
    wget \
    gcc \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-default-generators \
    ros-humble-ament-cmake-auto \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directories
WORKDIR /root/ws_livox/src
COPY ./livox_ros_driver2 ./livox_ros_driver2
WORKDIR /root
COPY ./Livox_SDK2 ./Livox_SDK2
WORKDIR /root/ws_fast_lio/src
COPY ./FAST_LIO ./FAST_LIO

# build Livox SDK
WORKDIR /root/Livox_SDK2
RUN mkdir build && cd build && cmake .. && make -j && make install

# build livox_ros_driver2
WORKDIR /root/ws_livox/src/livox_ros_driver2
RUN chmod +x build.sh
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && \
    ./build.sh humble"

# build FAST_LIO
WORKDIR /root/ws_fast_lio
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source /root/ws_livox/install/setup.bash && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build --symlink-install && \
    . ./install/setup.bash"

# Set up environment
WORKDIR /root
COPY ./docker-entrypoint.sh /root/docker-entrypoint.sh
RUN chmod +x /root/docker-entrypoint.sh

ENTRYPOINT [ "/root/docker-entrypoint.sh" ]
#ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /opt/ros/humble/setup.sh && /root/ws_livox/src/livox_ros_driver2/build.sh humble && source /root/ws_livox/install/setup.bash && source /root/ws_livox/install/setup.sh && exec \"$@\"", "--"]
CMD ["/bin/bash"]


# Still need to somehow allow display to work