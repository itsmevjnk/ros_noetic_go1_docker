FROM ros:noetic-ros-base-focal

# expose ROS port (not sure if this is needed)
# EXPOSE 11311

# keep our image updated
RUN apt-get update && apt-get upgrade -y

# build workspace
ENV ROS_WS=/opt/unitree_ws
WORKDIR $ROS_WS
COPY ./src ./src
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd $ROS_WS; rosdep install --from-paths src --ignore-src -y; catkin_make'

# clean up
RUN rm -rf /var/lib/apt/lists/*

# set up entry point
RUN sed --in-place --expression '$isource "$ROS_WS/devel/setup.bash"' /ros_entrypoint.sh
CMD ["roslaunch", "unitree_ros_io", "unitree_start.launch"]