FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt upgrade -y

RUN apt install -y cmake build-essential python3-pip python3-dev curl
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update

# --- Install ROS Packages --- #
RUN apt-get update && apt-get install -y ros-noetic-desktop

# --- ROS Post-Installation Procedures --- #
RUN apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN rosdep init
RUN rosdep update

# --- PyTorch and Deep Learning Libraries --- #
RUN pip3 install torch torchvision ultralytics pandas seaborn tqdm matplotlib nptyping beartype

WORKDIR /project
