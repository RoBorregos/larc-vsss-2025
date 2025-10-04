ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG TORCH_INDEX

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install Nav2 and common dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
# TODO ----Install ros2 dependencies ----
    python3-colcon-common-extensions \
    ros-humble-odom-to-tf-ros2 \
    ros-humble-urdf-launch \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Optional dev tools
RUN pip install --upgrade pip
RUN apt-get remove -y python3-sympy
RUN pip install torch torchvision ${TORCH_INDEX}
RUN pip install typing_extensions numpy pillow transforms3d scipy 
RUN pip install --no-deps ultralytics

# Setup ROS workspace directory and permissions
RUN mkdir -p /ros/vsss_ws/src && \
    chown -R ros:ros /ros

# Clean any existing rosdep data and initialize rosdep (run as root)
RUN rm -rf /etc/ros/rosdep/sources.list.d/* /var/lib/rosdep/* && \
    rosdep init && \
    rosdep fix-permissions && \
    rosdep update

# Switch to non-root user
RUN usermod -aG video ros
USER ros
WORKDIR /ros/vsss_ws

# Update rosdep db and install dependencies (run as ros user)
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

CMD ["bash"]

# Source ROS 2 setup on login
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
