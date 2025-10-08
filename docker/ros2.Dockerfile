FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY requirements.txt /workspace/requirements.txt
RUN pip3 install --no-cache-dir -r requirements.txt

COPY . /workspace/project

# Build the ROS 2 workspace
WORKDIR /workspace/project/ros2_ws
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select neuromorphic_navigation_ros

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source install/setup.bash && export NEUROMORPHIC_SIM_ROOT=/workspace/project && ros2 launch neuromorphic_navigation_ros simulation_bridge.launch.py"]
