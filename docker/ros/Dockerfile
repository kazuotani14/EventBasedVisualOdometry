FROM evo/tools

# Dockerfile maintainer
MAINTAINER Kazuya Otani, <kazuotani14@gmail.com>

USER root

ENV ROS_DISTRO kinetic
ENV CATKIN_SRC_DIR /home/camerauser/catkin_ws/src

# Install ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

USER camerauser

# Install ROS Desktop Full
RUN sudo apt-get update
RUN sudo apt-get install -y \
			ros-$ROS_DISTRO-ros-base \
			ros-$ROS_DISTRO-rosdoc-lite \
			python-catkin-lint \
			python-rosinstall \
			ros-$ROS_DISTRO-common-msgs \
			ros-$ROS_DISTRO-tf2-ros \
			ros-$ROS_DISTRO-xacro \
			ros-$ROS_DISTRO-tf \
			ros-$ROS_DISTRO-roscpp \
			ros-$ROS_DISTRO-robot-state-publisher \
			ros-$ROS_DISTRO-joint-state-publisher \
			ros-$ROS_DISTRO-rviz \
			ros-$ROS_DISTRO-tf2-eigen\
			ros-$ROS_DISTRO-rqt\
			ros-$ROS_DISTRO-rqt-common-plugins\
			ros-$ROS_DISTRO-rqt-robot-plugins

# Initialise rosdep
RUN sudo rosdep init && rosdep update
# Set up environment
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

RUN /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash"

# Setup .bashrc for ROS
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
    #Fix for qt and X server errors
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc
    # cd to home on login
RUN echo "cd" >> /home/$USERNAME/.bashrc

RUN mkdir -p $CATKIN_SRC_DIR
RUN  /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash; \
	cd $CATKIN_SRC_DIR; \
	catkin_init_workspace; \
	catkin init; \
	rm CMakeLists.txt; \
	cp /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake CMakeLists.txt;"

USER camerauser
