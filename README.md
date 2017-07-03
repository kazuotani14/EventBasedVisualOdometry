# EVO

Matlab/C++ implementation of Event-Based Multi-View Stereo (mapping half of Event-Based Visual Odometry)

<!-- ![](https://github.com/kazuotani14/EventBasedVisualOdometry) -->

### Prerequisites

_Matlab_: standalone - just need to get data.

_ROS node_:

* OpenCV
* Eigen
* [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros)

### Data

Get datasets from University of Zurich's [Event-Camera Dataset].

_Matlab_: download data in txt format, and run ExtractData on it to get a mat file.

_ROS node_: download rosbag to data folder and point emvs.launch to it.

### Running

_Matlab_: Run EMVS.m

_ROS node_: `roslaunch evo emvs.launch`

### References

* Rebecq, Henri, Guillermo Gallego, and Davide Scaramuzza. ”EMVS: Event-based Multi-View Stereo.” British Machine Vision Conference (BMVC). No. EPFL-CONF-221504. 2016.

* Henri Rebecq, Timo Horstschaefer, Guillermo Gallego, Davide Scaramuzza, "EVO: A Geometric Approach to Event-based 6-DOF Parallel Tracking and Mapping in Real-time," IEEE Robotics and Automation Letters (RA-L), 2016.

* Gallup, David, et al. ”Real-time plane-sweeping stereo with multiple sweeping directions.” Computer Vision and Pattern Recognition, 2007. CVPR’07. IEEE Conference on. IEEE, 2007
