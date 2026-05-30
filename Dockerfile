# Specify the platform as linux/amd64 or linux/arm64 based on your system
FROM osrf/ros:humble-desktop

# Install SO dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    build-essential \
    nano \
    python3-pip \
    python3-tk \
    gedit \
    terminator \
    gosu \
    --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# Pin NumPy 1.x and install SciPy via pip. Unpinned pip installs pull NumPy 2.x while
# Ubuntu's apt SciPy (1.8) stays on sys.path and breaks (AttributeError: _ARRAY_API).
RUN pip install --no-cache-dir \
    "numpy>=1.21,<2" \
    "scipy>=1.10" \
    pyserial tqdm \
    "opencv-python<4.12" \
    "matplotlib>=3.9" \
    rosbags numba
    
# Install ROS dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    alsa-utils \
    software-properties-common \
    libgflags-dev \
    ros-humble-test-msgs \
    ros-humble-rosbag2-storage-mcap \
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
RUN echo "export MPLCONFIGDIR=/home/ws/.cache/matplotlib" >> /etc/bash.bashrc

# Host UID/GID mapping entrypoint
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN sed -i 's/\r//' /usr/local/bin/entrypoint.sh && chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]