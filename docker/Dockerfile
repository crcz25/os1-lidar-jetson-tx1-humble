##############
# Base image #
##############
FROM dustynv/ros:humble-desktop-pytorch-l4t-r35.4.1 AS base

# Set the default shell to bash rather than sh
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Update, upgrade and clean packages
RUN apt-get update  \
    && apt-get upgrade -y \
#    && apt-get dist-upgrade -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Configure locale
# Install Locale
RUN apt-get update \
&& apt-get install -y locales \
&& rm -rf /var/lib/apt/lists/*

# Set locale
RUN locale-gen en_US en_US.UTF-8 \
&& update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
&& export DEBIAN_FRONTEND=noninteractive \
&& apt-get update \
&& apt-get install -y tzdata \
&& dpkg-reconfigure --frontend noninteractive tzdata \
&& rm -rf /var/lib/apt/lists/*

# Set the default locale
ENV LANG=en_US.UTF-8

# Install ROS2 packages from source
COPY docker/ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
RUN sudo chmod +x /usr/local/bin/ros_entrypoint.sh

COPY docker/ros2_install.sh /usr/local/bin/ros2_install.sh
RUN sudo chmod +x /usr/local/bin/ros2_install.sh

RUN /usr/local/bin/ros2_install.sh https://github.com/ros-perception/perception_pcl.git ros2
RUN /usr/local/bin/ros2_install.sh https://github.com/ouster-lidar/ouster-ros.git ros2

# Install additional packages
RUN apt-get update \
    && apt-get -y install \
    python3-colcon-argcomplete \
    python3-colcon-common-extensions \
    python3-osrf-pycommon \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Install additional tools
RUN apt-get update \
&& apt-get install -y \
   ack \
   cloc \
   gdb \
   htop \
   iperf3 \
   iputils-ping \
   mlocate \
   net-tools \
   psmisc \
   sudo \
   nano \
&& rm -rf /var/lib/apt/lists/*

##############
#  DEV image #
##############
FROM base AS dev
ARG AMENT_WORKSPACE_DIR="/ament_ws"
ARG USERNAME=devuser
ARG PASSWORD=devuser
ARG USER_UID=1000
ARG USER_GID=1000

# Set the default shell to bash rather than sh
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown -R $USER_UID:$USER_GID /home/$USERNAME/.config

# Setup sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*
# Create XDG_RUNTIME_DIR directory (run/user/UID) and set permissions (0700)
RUN mkdir -p /run/user/${USER_UID} \
    && sudo chown -R ${USER_UID}:${USER_GID} /run/user/${USER_UID} \
    && sudo chmod 0700 /run/user/${USER_UID}
# Setup the workspace
RUN mkdir -p ${AMENT_WORKSPACE_DIR}/src \
    && chown -R ${USER_UID}:${USER_GID} /home/${USERNAME} \
    && chown -R ${USER_UID}:${USER_GID} ${AMENT_WORKSPACE_DIR}
# Change password
RUN echo "${USERNAME}:${PASSWORD}" | sudo chpasswd

###############
#  Full image #
###############
FROM dev AS full

ARG OS_HN="os-XXXXXXXXXXXX.local"
ARG OS_IP="XXX.XXX.XXX.XXX"

# Set the renamed user as the default user for the container
USER $USERNAME

### Setup .bashrc ###
ENV DEBIAN_FRONTEND=dialog
# Add alias for ros sources
RUN echo "alias src_ros='source /opt/ros/${ROS_DISTRO}/install/setup.bash'" >> /home/$USERNAME/.bashrc
# Add completion of vcstool
RUN echo "source /usr/share/vcstool-completion/vcs.bash" >> /home/$USERNAME/.bashrc
# Add source of ROS
RUN echo "source /opt/ros/${ROS_DISTRO}/install/setup.bash" >> /home/$USERNAME/.bashrc
# FIX setuptools for colcon
RUN echo 'export PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"' >> /home/$USERNAME/.bashrc
# Build the workspace
RUN echo "alias build_ws='colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release' -Wall -Wextra -Wpedantic && \
    source /ament_ws/install/setup.bash'" >> /home/$USERNAME/.bashrc

# Set entrypoint
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN sudo chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/bin/bash", "/usr/local/bin/entrypoint.sh"]

# Exec the shell
CMD ["/bin/bash"]
