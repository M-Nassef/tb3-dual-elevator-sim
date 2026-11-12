# ROS 2 Jazzy + Gazebo Harmonic + Nav2 + TurtleBot3 for the dual-elevator simulation
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-nav2-simple-commander \
      ros-jazzy-turtlebot3-description \
      ros-jazzy-turtlebot3-gazebo \
      ros-jazzy-turtlebot3-navigation2 \
      python3-pytest python3-yaml python3-numpy python3-pil \
      mesa-utils git \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    QT_X11_NO_MITSHM=1
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc
WORKDIR /ws
