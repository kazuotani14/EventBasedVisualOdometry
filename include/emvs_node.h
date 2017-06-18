#ifndef _EMVS_NODE_H_
#define _EMVS_NODE_H_

#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

//OpenCV
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "keyframe_dsi.h"
#include "filters.h"
#include "opencv_defs.h"
#include "matrix_utils.h"

namespace emvs{

// TODO read camera info from topic
static int sensor_rows = 180;
static int sensor_cols = 240;
cv::Mat D_camera = (cv::Mat_<double>(5,1) << -0.3684363117977873, 0.1509472435566583, -0.0002961305343848646, -0.000759431726241032, 0.0);
cv::Mat K_camera = (cv::Mat_<double>(3,3) << 199.0923665423112, 0.0, 132.1920713777002, 0.0, 198.8288204700886, 110.7126600112956, 0.0, 0.0, 1.0);

double min_depth = 0.7;
double max_depth = 1.5;
double N_planes = 50;
double fx = 199.0923665423112;
double fy = 198.8288204700886;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Just put algorithm in here for now, separate later
class EmvsNode
{
public:
	EmvsNode();
	~EmvsNode();

private:
	ros::NodeHandle nh_;
	ros::Subscriber events_sub_;
	ros::Subscriber camera_info_sub_;
	ros::Subscriber ground_truth_sub_;
	ros::Subscriber imu_sub_;

	ros::Publisher pointcloud_pub_;
	PointCloud map_points_;

	// sensor_msgs::CameraInfo camera_info_;
	// bool camera_info_received_;

	KeyframeDSI kf_dsi_;
	bool first_;
	bool events_updated_;

	cv::Mat latest_event_image_;
	cv::Mat new_event_image_;
	geometry_msgs::PoseStamped latest_pose_estimate_;

	static constexpr double new_kf_dist_thres_ = 0.05; //[m]
	Eigen::Vector3d kf_pos_;
	Eigen::Vector4d kf_quat_;

	void eventCallback(const dvs_msgs::EventArray& msg);
	void poseCallback(const geometry_msgs::PoseStamped& msg);
	// void camerainfoCallback(const sensor_msgs::CameraInfo& msg);
	cv::Mat undistortImage(const cv::Mat input_image);
	void updateDsi(cv::Mat event_img);
	void addDsiToMap();
	bool checkForNewKeyframe(const geometry_msgs::PoseStamped& pose);
};

} // end namespace emvs

#endif
