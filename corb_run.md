# Guide to building and running corb slam

## Building

1) Clone repo

    ```bash
    git clone https://github.com/d-vo/collab_orb_slam2
    cd collab_orb_slam2
    ```

2) Create bs.sh

    ```bash
    source /opt/ros/melodic/setup.bash
    export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/collab_orb_slam2/Examples/ROS
    ```

3) Create Dockerfile

    ```Dockerfile
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
        libeigen3-dev 
    COPY . collab_orb_slam2
    WORKDIR /collab_orb_slam2/
    RUN set -x && \
        chmod +x build.sh && \
        ./build.sh
    RUN chmod +x bs.sh
    RUN /bin/bash -c 'source bs.sh && chmod +x build_ros_compress.sh && ./build_ros_compress.sh'
    ENTRYPOINT ["/bin/bash"]
    ```

4) Build image

    ```bash
    docker build --rm -f "Dockerfile" -t collab_orb_slam2:latest .
    ```

## Running

- Run `xhost local:` in bash if it's your first time running the server since your last boot.

### Server

First, run this command to start the server.  You'll have to make sure to include the path to the dataset directory.

```bash
docker run -it --rm --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro -v ~/orb_slam2/kitti/dataset/:/collab_orb_slam2/dataset collab_orb_slam2:latest
```

Next, within the container, run the following:

```bash
source bs.sh
roscore &
```

Wait a few seconds and then hit CTRL^C.  Then,run

```bash
./Examples/ROS/compression/KittiServerDepth -v Vocabulary/voc_k10_l_5_N_100000.txt -c Vocabulary/stats_8b.vstats -s Examples/ROS/compression/
```

### Agent 0

Start the agent

```bash
docker run -it --rm --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro -v ~/orb_slam2/kitti/dataset/:/collab_orb_slam2/dataset collab_orb_slam2:latest
```

Run the following

```bash
source bs.sh
./Examples/ROS/compression/KittiAgentDepth -v Vocabulary/voc_k10_l_5_N_100000.txt -c Vocabulary/stats_8b.vstats -i dataset/sequences/00 -r 0 -s  Examples/ROS/compression/KITTI00-02.yaml
```

### Agent 7

Start the agent

```bash
docker run -it --rm --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro -v ~/orb_slam2/kitti/dataset/:/collab_orb_slam2/dataset collab_orb_slam2:latest
```

Run the following

```bash
source bs.sh
./Examples/ROS/compression/KittiAgentDepth -v Vocabulary/voc_k10_l_5_N_100000.txt -c Vocabulary/stats_8b.vstats -i dataset/sequences/07 -r 1 -s  Examples/ROS/compression/KITTI04-12.yaml
```
