# Specify the platform as linux/amd64 or linux/arm64 based on your system
FROM osrf/ros:humble-desktop

# Install SO dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    build-essential \
    nano \
    python3-pip \
    gedit \
    terminator \
    gosu \
    --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip install pyserial tqdm numpy opencv-python scipy matplotlib rosbags numba
    
# Install ROS dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    alsa-utils \
    software-properties-common \
    libgflags-dev \
    ros-humble-test-msgs \
    libdw-dev \
    libacl1-dev \
    udev \
    && rm -rf /var/lib/apt/lists/*
    
# Optional dependencies:
# 435Le writeCustomerDate feature:
# RUN apt-get update -qq && apt-get install -y libssl-dev && rm -rf /var/lib/apt/lists/*

# Create workspace directory
RUN mkdir -p /home/ws

# Set working directory
WORKDIR /home/ws

# Source ROS setup files for all interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
RUN echo "if [ -f /home/ws/install/setup.bash ]; then source /home/ws/install/setup.bash; fi" >> /etc/bash.bashrc

# Host UID/GID mapping entrypoint
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]