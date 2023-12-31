# multi-stage for caching
FROM ros:humble

# This variable tells future scripts that user input isn't available during the Docker build.
ENV DEBIAN_FRONTEND noninteractive

LABEL maintainer="Nova"

# apt-get & apt installs
# Also need to upgrade the apt packages, otherwise there are incompatibilities running ros2/rviz2
RUN apt-get update && apt update && apt upgrade -y && \
    # =================================
    ## apt-get installs ===============
    apt-get install -y \
        # important linux tool
        wget \  
        # install python libraries
        python3-pip \
        # be able to use git inside container
        git \
        #
        fprintd \
        #
        libfprint-2-2:amd64 \
        #
        libgl1-mesa-glx \
        #
        libglib2.0-0 \
        #
        libpam-fprintd:amd64 \
        #
        libsm6 \
        #
        libxrender-dev \
        #
        libxext6 \
        #
        ninja-build \
        #
        ros-humble-diagnostic-updater \
    # =================================
    # apt installs ====================
    && apt install -y \
        #
        alsa-base \
        #
        alsa-utils \
        #
        byobu \
        #
        espeak \
        #
        ffmpeg \
        #
        libespeak1 \
        #
        libsndfile1-dev \
        #
        minicom \
        #
        python3-matplotlib \
        #
        python3-nose \
        #
        python3-scipy \
        #
        ros-humble-ament-cmake* \
        # needed for ros2_numpy package (maybe others)
        ros-humble-ament-cmake-nose \
        #
        ros-humble-async-web-server-cpp \
        #
        ros-humble-cv-bridge \
        #
        ros-humble-image-transport \
        #
        ros-humble-joy-linux \
        #
        ros-humble-lgsvl-msgs \
        #
        ros-humble-pcl-conversions \
        # Point Cloud Library (for lidar)
        ros-humble-pcl-ros \
        #
        ros-humble-rmw-fastrtps-cpp \
        #
        ros-humble-robot-localization \
        #
        ros-humble-rosbridge-server \
        #
        ros-humble-ros-testing \
        # rqt provides oscilliscope-style plotting of topics, etc
        ros-humble-rqt* \
        #
        ros-humble-rviz2 \
        #
        ros-humble-tf-transformations \
        # ROS Support for Velodyne LiDAR
        ros-humble-velodyne-driver \
        ros-humble-velodyne-laserscan \
        ros-humble-velodyne-msgs \
        ros-humble-velodyne-pointcloud \
        # Needed for Ouster lidar:
        ros-humble-tf2-eigen \
        build-essential \
        libeigen3-dev \
        libjsoncpp-dev \
        libspdlog-dev \
        libcurl4-openssl-dev \
        cmake \
        python3-colcon-common-extensions \
        #
        ros-humble-vision-opencv \
        #
        software-properties-common \
    # cleanup to make image smaller
    && apt clean && rm -rf /var/lib/apt/lists/*

# pip3 installs
RUN pip3 install \
    #
    autopep8 \
    #
    cmake_format==0.6.11 \
    #
    dictor \
    #
    ephem \
    #
    matplotlib \
    #  (possibly needs to be version 2.2)
    networkx \
    # Scientific Computing - used widely
    numpy \
    #
    open3d \
    #
    opencv-python \
    #
    pep8 \
    #
    pexpect \
    #
    psutil \
    #
    pygame \
    #
    pylint \
    #
    pymap3d==2.9.1 \
    #
    pynmea2 \
    #
    pyproj \
    #
    pyserial \
    #
    python-can \
    #
    py-trees==0.8.3 \
    #
    requests \
    #
    scikit-image \
    # Scientific Computing - used widely
    scipy \
    #
    shapely==2.0.0 \
    #
    simple-pid \
    #
    simple-watchdog-timer \
    #
    six \
    #
    tabulate \
    #
    transforms3d \
    #
    xmlschema==1.0.18
    # distro \
    # torch==1.12.0 \
    # torchvision \
    # openmim \
    # mmcv-full==1.6.0 \
    # packaging \
    # prettytable \
    # python-dateutil>=2.1 \
    # kiwisolver>=1.0.1 \
    # pyparsing>=2.0.1 \
    # cycler>=0.10 \
    # wcwidth \
    # six>=1.5 \
    # mmcls \
RUN pip3 install --upgrade numpy && pip3 install --upgrade scipy

# https://stackoverflow.com/questions/66669735/ubuntu-20-04-cant-find-pcl-because-of-incorrect-include-directory-after-install
RUN mkdir /lib/x86_64-linux-gnu/cmake/pcl/include && ln -s /usr/include/pcl-1.10/pcl /lib/x86_64-linux-gnu/cmake/pcl/include/pcl

# Lemon
# Library for Efficient Modeling and Optimization in Networks. It is a C++ template library providing efficient implementations of common data structures and algorithms with focus on combinatorial optimization tasks connected mainly with graphs and networks.
RUN wget http://lemon.cs.elte.hu/pub/sources/lemon-1.3.1.tar.gz && tar xvzf lemon-1.3.1.tar.gz && cd lemon-1.3.1 && mkdir build && cd build && cmake .. && make && make install

# Download and build Ouster ROS Drivers
RUN mkdir /drivers
WORKDIR /drivers
RUN git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
WORKDIR /drivers/ouster-ros
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

# RUN useradd -ms /bin/bash docker
RUN usermod -a -G dialout root
RUN usermod -a -G tty root
# USER docker

ENV ROS_VERSION 2

WORKDIR /vehicle_interface
COPY ./docker/entrypoint.sh /opt/entrypoint.sh

ENTRYPOINT [ "/opt/entrypoint.sh" ]