# Use the official Ubuntu 22.04 image
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    git \
    lsb-release \
    gnupg2 \
    locales \
    software-properties-common \
    python3-pip \
    python3-opencv \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# Install WSL2 tools (if running in WSL, else this step is ignored)
RUN apt-get update && apt-get install -y \
    linux-tools-common \
    linux-tools-generic \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo 11
RUN apt-get update && apt-get install -y \
    gazebo11 \
    libgazebo11-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create a workspace
RUN mkdir -p /ros2_ws/src

# Clone your GitHub repository into the workspace
RUN git clone <YOUR_GITHUB_REPO_URL> /ros2_ws/src/your_repo

# Install Python dependencies from requirements.txt
COPY requirements.txt /ros2_ws/requirements.txt
RUN pip3 install --no-cache-dir -r /ros2_ws/requirements.txt

# Build the ROS 2 workspace
WORKDIR /ros2_ws
RUN . /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install

# Source the ROS 2 workspace in .bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Command to keep the container running and ready for interactive use
CMD ["bash"]