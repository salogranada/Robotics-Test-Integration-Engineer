# -----------------------------------------------------------------------------
# dadaroce (davidson@kiwibot.com)
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
FROM ubuntu:20.04

# Or your actual UID, GID on Linux if not the default 1000
ARG USERNAME=koda
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND noninteractive

# -----------------------------------------------------------------------------
# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # ------------------------------
    # Free memory
    && apt autoremove -y && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Install essentials
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # ------------------------------
    # Basics
    apt-utils \
    bash-completion \
    build-essential \
    htop \
    lsb-release \
    openssh-client \
    # ------------------------------
    # Download repositories and files
    apt-transport-https \
    ca-certificates \
    curl \
    git \
    gnupg-agent \
    gnupg2 \
    iproute2 \
    wget \
    # ------------------------------
    # Network stuff
    lsb-release \
    network-manager \
    wireless-tools \
    # ------------------------------
    # Install clang formater
    clang-format \
    # ------------------------------
    # Parsing yaml in c++
    libyaml-cpp-dev \
    # ------------------------------
    # Python
    python3 \
    python3-argcomplete \
    python3-distutils \
    python3-pip \
    python3-tk \
    # ------------------------------
    # Audio tools
    alsa-base \
    alsa-tools \
    alsa-utils \
    libasound2-dev \ 
    pulseaudio \
    # ------------------------------
    # Debug tools
    valgrind \
    # ---------------------------------
    && apt autoremove -y && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Install cmake from source, because latest version not available in apt repos
ENV CMAKE_VERSION=3.22
RUN wget https://cmake.org/files/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}.0-linux-x86_64.sh \
    && bash cmake-${CMAKE_VERSION}.0-linux-x86_64.sh --skip-license --prefix=/usr/local

# -----------------------------------------------------------------------------
# Installing ros2
# https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Install-Debians/
ENV ROS_DISTRO=galactic

# Editors and so on
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \ 
    && apt update && apt install --no-install-recommends -y \ 
    # Base Installation (Bare Bones): Communication libraries, message packages.
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rclcpp-cascade-lifecycle \
    ros-${ROS_DISTRO}-rqt* \
    python3-colcon-common-extensions \
    # ------------------------------
    && apt autoremove && apt clean -y \
    && rm -rf /var/lib/apt/lists/* \
    # ------------------------------
    # Rosedep
    && pip3 install rosdep \ 
    && rosdep init \
    && rosdep update \
    # ROS2 source setup
    && ROS_SOURCE="source /opt/ros/${ROS_DISTRO}/setup.sh" \
    && pip3 install -U argcomplete \
    && rm -rf /home/koda/.cache/pip

# -----------------------------------------------------------------------------
# Python Libraries
RUN pip3 install -U pip \
    && python3 -m pip install \
    # Add EXTRA DEV Python dependencies in here
    # ------------------------------
    # Tools to check code compliance
    typed-ast \
    black \
    click \
    pylint~=2.11.1 \
    # ------------------------------
    # Installing basics
    setuptools \
    # ------------------------------
    # Plotter Tools
    matplotlib\
    && rm -rf /home/koda/.cache/pip

# ------------------------------------------------------------------------------
# ROS2 CONFIGS - ROS2 CONFIGS - ROS2 CONFIGS - ROS2 CONFIGS - ROS2 CONFIGS - ROS2

# Logging variables
# https://index.ros.org/doc/ros2/Tutorials/Logging-and-logger-configuration/
# # If you would like more or less verbose formatting
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}][{time}][{name}]: {message}"
# If you would like to force enabling or disabling it
# the output is colorized when it’s targeting a terminal. 
ENV RCUTILS_COLORIZED_OUTPUT=1                                   
# By default, info and debug log calls aren’t line buffered. 
# You can force it using 
# ENV RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1   # deprecated
ENV RCUTILS_LOGGING_USE_STDOUT=1   
ENV RCUTILS_LOGGING_BUFFERED_STREAM=1

# -----------------------------------------------------------------------------
# disable the creation of core dump files on a UNIX or Linux environment
# https://dev.intelrealsense.com/docs/troubleshooting
# https://www.ibm.com/support/pages/how-disable-creation-core-dump-files-unix-or-linux
ENV ulimit -c 0

# -----------------------------------------------------------------------------
# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=
CMD ["/bin/bash"]