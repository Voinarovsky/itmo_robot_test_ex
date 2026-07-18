FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# --- системные и ROS-зависимости -------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      nano \
      python3-pip \
      python3-colcon-common-extensions \
      ros-humble-xacro \
      ros-humble-moveit \
      ros-humble-moveit-msgs \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-ros2-control \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-joint-state-publisher \
      ros-humble-robot-state-publisher \
      ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# --- исходники робота -------------------------------------------------------
# ВАЖНО: aubo_description — git-submodule, без --recurse-submodules папка пустая
WORKDIR /ros2_ws/src
RUN git clone --recurse-submodules \
      https://github.com/AuboRobot/aubo_ros2_driver.git

# --- наши доработки поверх --------------------------------------------------
COPY patches/ /ros2_ws/src/aubo_ros2_driver/

# --- сборка -----------------------------------------------------------------
# НЕ --symlink-install: в репо битые симлинки мешей (aubo_i10*), colcon падает
# aubo_ros2_driver (реальный робот) не собираем — требует pyaubo_sdk
WORKDIR /ros2_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-up-to aubo_gazebo aubo_moveit_config

# --- скрипты ----------------------------------------------------------------
COPY scripts/ /ros2_ws/scripts/
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/run_demo.sh   /ros2_ws/scripts/run_demo.sh
RUN chmod +x /entrypoint.sh /ros2_ws/scripts/run_demo.sh

ENV ROS_DOMAIN_ID=42

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
