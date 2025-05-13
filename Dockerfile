# Start from official ROS 2 Iron base image
FROM ros:iron-ros-base

# Avoid interactive prompts during install
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=89

# Install system dependencies
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-opencv \
    git \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install latest compatible Python packages
RUN python3 -m pip install --upgrade pip setuptools wheel setuptools==65.5.1

# Install latest stable versions of numpy and matplotlib
RUN pip install \
    numpy \
    matplotlib \
    torch \
    stable-baselines3 \
    opencv-python \
    pybind11

# Set working directory
WORKDIR /root/ros2_ws

# Copy your ROS 2 package source code
COPY ./src ./src

# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/iron/setup.bash && colcon build"

# Default command to run your PPO node
CMD ["/bin/bash", "-c", "source /opt/ros/iron/setup.bash && source install/setup.bash && ros2 run autonomous_rov ppo_control_node"]
