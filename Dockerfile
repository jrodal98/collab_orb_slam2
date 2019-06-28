FROM osrf/ros:melodic-desktop-full-bionic
RUN set -x && \
    apt update -y -qq && \
    apt upgrade -y -qq --no-install-recommends && \
    apt install -y -qq \
    build-essential \
    pkg-config \
    cmake \
    git \
    wget \
    curl \
    tar \
    unzip \
    libglew-dev \
    libboost-all-dev \
    libopencv-dev \
    build-essential \
    cmake \
    cmake-gui \
    libeigen3-dev && \
    apt autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

# protobuf
WORKDIR /tmp
RUN set -x && \
    wget -O - https://github.com/google/protobuf/archive/v3.6.1.tar.gz | tar zxvf -&& \
    cd protobuf-3.6.1 && \
    ./autogen.sh && \
    ./configure --prefix=/usr/local --enable-static=no && \
    make -j${NUM_THREADS} && \
    make install && \
    cd /tmp && \
    rm -rf *
WORKDIR /
COPY . collab_orb_slam2
WORKDIR /collab_orb_slam2/
RUN set -x && \
    chmod +x build.sh && \
    ./build.sh
RUN chmod +x bs.sh
RUN /bin/bash -c 'source bs.sh && chmod +x build_ros_compress.sh && ./build_ros_compress.sh'
ENTRYPOINT ["/bin/bash"]