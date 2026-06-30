# syntax=docker/dockerfile:1

FROM osrf/ros:humble-desktop-full
ARG DEBIAN_FRONTEND=noninteractive
ARG HOME
ARG USER_NAME
ARG UID
ARG GID
ARG ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

# Install commonly-used tools.
RUN apt-get update && apt-get install --yes \
    curl \
    git \
    iproute2 \
    iputils-ping \
    less \
    mesa-utils \
    net-tools \
    nano \
    rsync \
    software-properties-common \
    sudo \
    tmux \
    tree \
    unzip \
    usbutils \
    vim \
    wget \
    zip \
    zsh


# Install the build tools
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && apt-get update
RUN apt-get update && apt-get install -y \
    build-essential \
    jq \
    python3-pip 

# Install the dependecies for CoHAN
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && apt install -y \
    ros-$ROS_DISTRO-navigation2\
    ros-$ROS_DISTRO-libg2o \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-joint-state-publisher \
    libeigen3-dev\
    libqt5gui5 \
    libsqlite3-dev \
    libqt5widgets5 \
    libzmq3-dev \
    libqt5core5a \
    qtbase5-dev \
    libsdl2-dev \
    libopenblas-dev \
    libsdl-image1.2-dev \
    libfreeimage-dev \
    && rm -rf /var/lib/apt/lists/*

## Add a non-root user and set proper permission
RUN addgroup --gid $GID $USER_NAME && \
    adduser --uid $UID --gid $GID --disabled-password --gecos "" $USER_NAME

## Add the user to sudo group and remove password requirement
RUN adduser $USER_NAME sudo && echo '%sudo ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers 

## Change user and copy the appropriate basrc
USER $USER_NAME
COPY ./bashrc /home/$USER_NAME/.bashrc

## Set the workdirectory (Don't forget to mount this in volumes)
WORKDIR $HOME

