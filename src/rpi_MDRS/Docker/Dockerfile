FROM osrf/ros:humble-desktop

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

RUN sudo apt-get update && apt-get install -y python3-colcon-common-extensions 
RUN sudo apt-get update && apt-get install -y ros-humble-rviz2
RUN sudo apt-get update && apt-get install -y ros-humble-controller-manager
RUN sudo apt-get update && apt-get install -y ros-humble-robot-state-publisher
RUN sudo apt-get update && apt-get install -y ros-humble-joint-state-publisher
RUN sudo apt-get update && apt-get install -y ros-humble-joint-state-publisher-gui
RUN sudo apt-get update && apt-get install -y ros-humble-gazebo-ros-pkgs
RUN sudo apt-get update && apt-get install -y ros-humble-trajectory-msgs
RUN sudo apt-get update && apt-get install -y ros-humble-velocity-controllers
RUN sudo apt-get update && apt-get install -y ros-humble-joint-trajectory-controller
RUN sudo apt-get update && apt-get install -y ros-humble-gazebo-ros2-control-demos
RUN sudo apt-get update && apt-get install -y ros-humble-teleop-tools 
RUN sudo apt-get update && apt-get install -y ros-humble-key-teleop
RUN sudo apt-get update && apt-get install -y ros-humble-tf-transformations
RUN sudo apt-get update && apt-get install -y ros-humble-xacro    

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

RUN sudo mkdir -p /workspace && sudo chown -R ${USERNAME}:${USERNAME} /workspace

USER ${USERNAME}

WORKDIR /workspace

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc


# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"] 
CMD ["bash"]

RUN sudo git clone https://github.com/SupaeroMoon-ERC-MDRS/OBC-MDRS-RPI















