FROM ros:humble-ros-base

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-matplotlib \
    python3-yaml \
    python3-numpy \
    ros-humble-sensor-msgs \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-py \
    ros-humble-rclpy \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-navigation2 \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Install pip-only Python dependencies
RUN pip3 install --no-cache-dir \
    apriltag \
    pymap3d

# Set working directory
WORKDIR /workspace

# Source ROS 2 environment automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Default command (override when running if needed)
CMD ["bash"]
