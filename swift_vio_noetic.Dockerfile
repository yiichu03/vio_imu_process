FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive

# 基础工具 + catkin/wstool/rosdep
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config \
    python3-catkin-tools python3-wstool python3-rosdep \
    libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libeigen3-dev libsuitesparse-dev \
    libboost-all-dev libtbb-dev \
    libopencv-dev libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# rosdep（容器里 init 可能会报已存在，忽略即可）
RUN rosdep init || true && rosdep update

# Sophus（按 README，需要源码安装）
RUN mkdir -p /opt/src && cd /opt/src && \
    git clone https://github.com/stevenlovegrove/Sophus.git && \
    cd Sophus && mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j"$(nproc)" && make install

# GTSAM（按 README 固定 commit + 开启 unstable）
RUN cd /opt/src && \
    git clone https://github.com/borglab/gtsam.git --recursive && \
    cd gtsam && git checkout 8c98eefb24f846267119f7f81466dd660f195b06 && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_USE_SYSTEM_EIGEN=ON && \
    make -j"$(nproc)" && make install

# 避免运行时找不到 /usr/local/lib
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/local.conf && ldconfig

WORKDIR /ws

