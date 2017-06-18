FROM evo/ros

USER camerauser

# Install prerequisites
RUN sudo apt-get install -y libusb-1.0-0-dev\
 					   ros-kinetic-camera-info-manager\
					   ros-kinetic-image-view\
					   python-catkin-tools

RUN sudo apt-get install -y libopencv-dev python-opencv

USER root

# Event-based camera stuff
RUN cd /home/camerauser/catkin_ws
RUN catkin config --init --mkdirs --extend /opt/ros/kinetic --merge-devel \
				  --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN cd /home/camerauser/catkin_ws/src; \
	git clone https://github.com/catkin/catkin_simple.git;\
    git clone https://github.com/uzh-rpg/rpg_dvs_ros.git

# Install our stuff
RUN cd /home/camerauser/catkin_ws/src; \
	git clone -b kazu_dev https://github.com/kazuotani14/EVO.git evo

RUN echo "source /home/camerauser/catkin_ws/devel/setup.bash" >> /home/camerauser/.bashrc

USER camerauser
