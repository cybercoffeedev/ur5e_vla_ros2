FROM docker.io/osrf/ros:jazzy-desktop-full

ARG ROS_DISTRO=jazzy
ARG USERNAME=ros2dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit this if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
ENV SHELL /bin/bash

# ---------------------------------------------------------------------------
# System dependencies.
#
# The ROS 2 packages of this project are deliberately NOT installed here:
# the UR driver, the Robotiq driver, ros2_control and the controllers all come
# from ur5e_vla.repos and are built from source (see setup-workspace.sh).
# Only generic tooling that is not project specific lives in the image.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    # vcs import -- the .repos workflow
    python3-vcstool \
    # tooling
    git curl wget vim tmux \
    python3-pip \
    # serial / USB diagnostics (RS-485 link of the gripper)
    python3-serial usbutils v4l-utils \
    # image processing (dataset recorder, policy bridge) and mp4 encoding (LeRobot)
    python3-opencv ffmpeg \
    # generic ROS tooling used by the launch files
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-rviz2 \
    # message package needed by the steering controllers of the ros2_controllers
    # metapackage (robotiq_description depends on that metapackage)
    ros-${ROS_DISTRO}-ackermann-msgs \
    && rm -rf /var/lib/apt/lists/*

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

RUN curl -fsSL https://opencode.ai/v2/install | bash -s -- --no-modify-path \
    && install -m 0755 /root/.opencode/bin/opencode /usr/local/bin/opencode \
    && opencode --version

# [Optional] Set the default user. Omit this if you want to keep the default as root.
USER $USERNAME
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo '[ -f /home/ws/install/setup.bash ] && source /home/ws/install/setup.bash' >> /home/$USERNAME/.bashrc

CMD ["/bin/bash"]
