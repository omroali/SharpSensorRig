FROM osrf/ros:jazzy-desktop-full

RUN apt-get update && apt-get install -y --no-install-recommends gnupg2 \
    && apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

RUN echo "deb http://packages.ros.org/ros2-testing/ubuntu $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/ros2-testing.list

ARG  DEBIAN_FRONTEND=noninteractive
ARG  USERNAME=ubuntu

RUN apt-get update && apt-get install -y sudo git usbutils udev \
    && addgroup plugdev 2>/dev/null || true \
    && addgroup video 2>/dev/null || true \
    && addgroup dialout 2>/dev/null || true \
    && usermod -aG sudo ${USERNAME} \
    && usermod -aG plugdev ${USERNAME} \
    && usermod -aG video ${USERNAME} \
    && usermod -aG dialout ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers


ENV BASE_WS=/home/${USERNAME}/base_ws
RUN mkdir -p ${BASE_WS}/src \
    && chown -R ${USERNAME}:${USERNAME} ${BASE_WS}

ENV BASH_SCRIPTS=/home/${USERNAME}/bash_scripts
RUN mkdir -p ${BASH_SCRIPTS} \
    && chown -R ${USERNAME}:${USERNAME} ${BASH_SCRIPTS}

ENV ROSBAG_DIR=/home/${USERNAME}/rosbags
RUN mkdir -p ${ROSBAG_DIR} \
    && chown -R ${USERNAME}:${USERNAME} ${ROSBAG_DIR}

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo git build-essential \
    gcc-12 g++-12 \
    python3 python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep python3-rosdep-modules && \
    rm -rf /var/lib/apt/lists/*

ENV CC=/usr/bin/gcc-12
ENV CXX=/usr/bin/g++-12

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    nvidia-cuda-toolkit && \
    rm -rf /var/lib/apt/lists/*

COPY docker/helper_math.h /usr/local/include/helper_math.h

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    vim nano htop tmux net-tools tree ffmpeg \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-libav && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-jazzy-launch-testing \
    ros-jazzy-tf-transformations \
    ros-jazzy-launch-pytest \
    ros-jazzy-ament-cmake-core \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-foxglove-bridge \
    libasio-dev \
    ros-jazzy-joy-linux \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-cv-bridge \
    ros-jazzy-nav-msgs \
    ros-jazzy-tf2-ros \
    && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get purge -y ros-jazzy-librealsense2 2>/dev/null || true && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    iproute2 can-utils \
    libturbojpeg0-dev \
    libusb-1.0-0-dev \
    cmake \
    pkg-config && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libomp-dev \
    libpcap-dev \
    freeglut3-dev \
    libglfw3-dev \
    libxmu-dev \
    libxi-dev \
    ocl-icd-opencl-dev \
    opencl-headers \
    ocl-icd-libopencl1 \
    clinfo \
    python3-uvicorn \
    python3-fastapi && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/OpenKinect/libfreenect2.git
WORKDIR /libfreenect2/build
RUN cmake .. -DENABLE_CXX11=ON -DBUILD_OPENNI2_DRIVER=OFF -DENABLE_OPENCL=OFF -DENABLE_CUDA=ON -DENABLE_OPENGL=OFF -DENABLE_VAAPI=OFF -DENABLE_TEGRAJPEG=OFF -DCMAKE_INSTALL_PREFIX=/usr
RUN make
RUN make install

RUN git clone https://github.com/realsenseai/realsense-ros.git \
    -b 4.57.7 --depth 1 ${BASE_WS}/src/realsense-ros \
    && chown -R ${USERNAME}:${USERNAME} ${BASE_WS}/src/realsense-ros

# Bring the host-installed RealSense SDK into the image so realsense-ros
# can be compiled at build time. Copy the whole lib/include/share trees so
# the SDK's cmake targets (rsutils, realdds, fastrtps, fastcdr,
# foonathan_memory, ...) resolve. At runtime these paths are shadowed by
# the read-only /usr/local bind mounts from the host (same 2.57.7 SDK).
COPY --from=rs_sdk lib /usr/local/lib
COPY --from=rs_sdk include /usr/local/include
COPY --from=rs_sdk share /usr/local/share
RUN ldconfig

# Copy the workspace package sources so they can be built into the image.
# At runtime these are overlaid by the read-write bind mounts in
# docker-compose.yml; with --symlink-install live Python edits take effect
# immediately, and C++ changes only need an incremental `wbuild`.
COPY --chown=${USERNAME}:${USERNAME} kinect2_ros2   ${BASE_WS}/src/kinect2_ros2
COPY --chown=${USERNAME}:${USERNAME} realsense_ros2 ${BASE_WS}/src/realsense_ros2
COPY --chown=${USERNAME}:${USERNAME} vicon_ros      ${BASE_WS}/src/vicon_ros
COPY --chown=${USERNAME}:${USERNAME} velodyne_ros   ${BASE_WS}/src/velodyne_ros
COPY --chown=${USERNAME}:${USERNAME} session_recorder ${BASE_WS}/src/session_recorder

RUN python3 -m pip install --no-cache-dir --break-system-packages \
    tmule==1.5.9 \
    pyserial

RUN mkdir -p /home/${USERNAME} && echo '{"context":{"dds":{"domain":0,"enabled":true}}}' \
    > /home/${USERNAME}/.realsense-config.json \
    && chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.realsense-config.json

USER ${USERNAME}
WORKDIR /home/${USERNAME}

RUN sed -i 's/^#\(force_color_prompt\)/\1/' /home/${USERNAME}/.bashrc

RUN echo '[ -f $HOME/bash_scripts/bashrc_extensions.sh ] && \
    source $HOME/bash_scripts/bashrc_extensions.sh' \
    >> /home/${USERNAME}/.bashrc

# Pre-build the workspace at image-build time so that starting a container
# (docker compose run --rm ...) no longer triggers a full colcon build.
# The compiler/env mirror the runtime values in docker-compose.yml so the
# baked build behaves identically to the previous on-start build.
# Source changes that need a rebuild can still be applied with `wbuild`.
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash \
    && cd ${BASE_WS} \
    && CC=/usr/bin/gcc-13 CXX=/usr/bin/g++-13 \
       CFLAGS=-fno-tree-fre CXXFLAGS=-fno-tree-fre MAKEFLAGS=-j1 \
       colcon build --symlink-install \
       --cmake-args -DCMAKE_BUILD_TYPE=Release \
       -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
    && rm -f install/COLCON_IGNORE"

CMD ["bin/bash"]
