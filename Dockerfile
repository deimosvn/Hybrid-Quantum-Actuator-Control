# Reproducible build + runtime image for the Hybrid Quantum Rover stack.
# Build:  docker build -t quantum_rover .
# Run  :  docker run --rm -it quantum_rover
#         (inside) ros2 launch quantum_rover_bringup simulation.launch.py
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# ---- System + ROS tooling --------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-scipy \
        python3-matplotlib \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# ---- Optional quantum backend (controller degrades to PID without it) ------
RUN pip3 install --no-cache-dir --break-system-packages \
        "qiskit>=1.0" "qiskit-aer>=0.14" || \
    pip3 install --no-cache-dir "qiskit>=1.0" "qiskit-aer>=0.14"

# ---- Workspace -------------------------------------------------------------
WORKDIR /ros_ws
COPY . /ros_ws/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --symlink-install --event-handlers console_direct+

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
