# This Dockerfile adds a catkin workspace to the
# indigo-ros-base image. The workspace is initialized
# at /catkin_ws.
FROM ros:indigo-ros-base

# Needed to suppress warning when installing g++:
# debconf: unable to initialize frontend: Dialog
# debconf: (TERM is not set, so the dialog frontend is not usable.)
# debconf: falling back to frontend: Readline
# ...
ENV DEBIAN_FRONTEND noninteractive

# Need to install g++ in order to enable catkin_ws
# creation (otherwise cmake fails with this error:
# CMake Error: your CXX compiler: "CMAKE_CXX_COMPILER-NOTFOUND" was not found.   Please set CMAKE_CXX_COMPILER to a valid compiler path or name.
# There is probably an easier way to fix this, but installing g++ did
# the job, even though gcc/catkin was already installed.
RUN apt-get update && apt-get install -y \
    g++ \
    vim \
    && rm -rf /var/lib/apt/lists/

# add environment setup 
RUN bin/bash -c "echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> .bashrc"

# Setup catkin workspace
RUN /bin/bash -c "apt-get update && \
                  source /opt/ros/$ROS_DISTRO/setup.bash && \
                  mkdir -p /catkin_ws/src && \
                  cd /catkin_ws/src && \
                  git clone https://github.com/AutonomyLab/create_autonomy.git && \
                  catkin_init_workspace && \
                  cd /catkin_ws/ && \
                  apt-get update && \
                  rosdep update && \
                  rosdep install --from-paths src -i -y &&\
                  catkin_make && \
                  echo 'source /catkin_ws/devel/setup.bash' >> /.bashrc && \
                  rm -rf /var/lib/apt/lists/"

COPY ./ros_entrypoint.sh /
RUN chmod +x ros_entrypoint.sh
COPY ./ros.launch /
CMD ["roslaunch", "--wait", "/ros.launch"]