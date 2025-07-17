FROM nvidia/cuda:12.6.1-base-ubuntu24.04
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO jazzy
ENV DEBIAN_FRONTEND=noninteractive
USER root
    RUN apt update -y && apt install locales -y
    RUN locale-gen en_US en_US.UTF-8
    RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    ENV LANG=en_US.UTF-8

    RUN apt install -y software-properties-common

    RUN apt install -y software-properties-common \
        && add-apt-repository universe -y

    RUN apt update -y && apt install curl -y
    RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
    RUN apt update -y && apt install -y ros-dev-tools
    RUN apt install -y python3-colcon-common-extensions
    RUN apt update -y &&\
        apt upgrade -y &&\
        apt install ros-jazzy-desktop-full -y &&\
        apt-get install python3-rosdep &&\
        rosdep init &&\
        rosdep update &&\
        rm -rf /var/lib/apt/lists/*

    RUN apt-get update -y &&\
        apt-get upgrade -y &&\
        apt-get install ros-${ROS_DISTRO}-gz-* -y &&\
        apt-get install ros-${ROS_DISTRO}-grid* -y &&\
        apt-get install ros-${ROS_DISTRO}-nav2* -y &&\
        apt-get install ros-${ROS_DISTRO}-rviz2* -y &&\
        apt-get install ros-${ROS_DISTRO}-teleop* -y &&\
        apt-get install ros-${ROS_DISTRO}-robot-state-publisher -y &&\
        apt-get install ros-${ROS_DISTRO}-joint* -y &&\
        apt-get install ros-${ROS_DISTRO}-control* -y &&\
        apt-get install ros-${ROS_DISTRO}-irob* -y &&\
        apt-get install ros-${ROS_DISTRO}-ament* -y &&\
        apt-get install ros-${ROS_DISTRO}-rviz* -y &&\
        rm -rf /var/lib/apt/lists/*

    # disable sudo password for ubuntu user
    ENV USER=ubuntu
    RUN usermod -aG sudo ubuntu
    RUN apt-get install -y sudo &&\
        echo ubuntu ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/ubuntu \
        && chmod 0440 /etc/sudoers.d/ubuntu \
        && chown -R ubuntu:ubuntu /home/ubuntu \
        && chmod -R 777 /home/ubuntu

USER ubuntu
    WORKDIR /home/ubuntu
        COPY docker/entrypoint.sh /home/ubuntu/entrypoint.sh

USER root
    WORKDIR /home/ubuntu
        RUN chmod +x /home/ubuntu/entrypoint.sh

USER ubuntu
    WORKDIR /home/ubuntu
        ENTRYPOINT ["/entrypoint.sh"]
    # create workspace