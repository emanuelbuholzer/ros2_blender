ARG ROS_DISTRO="humble"

FROM docker.io/ros:${ROS_DISTRO}

# See https://github.com/opencontainers/runc/issues/2517
RUN echo 'APT::Sandbox::User "root";' > /etc/apt/apt.conf.d/sandbox-disable

ENV ROS_OVERLAY /opt/ros/ros2_blender

WORKDIR $ROS_OVERLAY

COPY launch src/ros2_blender/launch
COPY resource src/ros2_blender/resource
COPY ros2_blender src/ros2_blender/ros2_blender
COPY test src/ros2_blender/test
COPY package.xml src/ros2_blender/package.xml
COPY setup.cfg src/ros2_blender/setup.cfg
COPY setup.py src/ros2_blender/setup.py

RUN apt-get update && \
    rosdep install -iy --from-paths src && \
    rm -rf /var/lib/apt/lists/

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon test --python-testing setuppy_test --event-handlers console_direct+ ; \
    colcon test-result --verbose

RUN sed --in-place --expression \
    '$isource "${ROS_OVERLAY}/install/setup.bash"' \
    /ros_entrypoint.sh