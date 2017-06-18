#include "emvs_node.h"

namespace emvs{

using cv::Mat;
using cv::Mat_;

EmvsNode::EmvsNode()
	: events_updated_(false),
      kf_dsi_(sensor_rows, sensor_cols, min_depth, max_depth, N_planes, fx, fy)
{
	events_sub_ = nh_.subscribe("dvs/events", 1000, &EmvsNode::eventCallback, this);
	ground_truth_sub_ = nh_.subscribe("optitrack/davis", 100, &EmvsNode::poseCallback, this);
	// camera_info_sub_ = nh_.subscribe("dvs/camera_info", 1, &EmvsNode::camerainfoCallback, this);

	pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("map_points", 1);
	map_points_.height = 1;

	latest_event_image_ = Mat::zeros(sensor_rows, sensor_cols, EVENT_IMAGE_TYPE);
	new_event_image_ = Mat::zeros(sensor_rows, sensor_cols, EVENT_IMAGE_TYPE);

	cv::namedWindow(OPENCV_WINDOW);
}

EmvsNode::~EmvsNode()
{
	cv::destroyWindow(OPENCV_WINDOW);
}

void EmvsNode::eventCallback(const dvs_msgs::EventArray& msg)
{
	// Add all events to DSI
	for(int i=0; i<msg.events.size(); i++)
	{
	    new_event_image_.at<uchar>(static_cast<int>(msg.events[i].y), static_cast<int>(msg.events[i].x)) += 1; // TODO this shoud be 1, but is 5000 for viz purposes
	}
	showNormalizedImage(new_event_image_);
}

void EmvsNode::poseCallback(const geometry_msgs::PoseStamped& msg)
{
	bool new_kf = checkForNewKeyframe(msg);

	if(new_kf && events_updated_)
	{
		addDsiToMap();
		kf_dsi_.resetDSI();
		events_updated_ = false;
	}
	else if (cv::countNonZero(new_event_image_) > 0)
	{
		// TODO Find out why event images aren't shown in the first few secs
		events_updated_ = true;
		latest_event_image_ = undistortImage(new_event_image_);
		updateDsi(latest_event_image_); // TODO this takes time, so put it on a separate thread?
		new_event_image_.setTo(0);
	}
}

// TODO figure out a better name that reflects the fact that this modifies members
bool EmvsNode::checkForNewKeyframe(const geometry_msgs::PoseStamped& pose)
{
	// check for new keyframe (position dist threshold)
	Eigen::Vector3d cur_pos;
	cur_pos << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
	Eigen::Vector4d cur_quat;
	cur_quat << pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w;
	double dist_from_kf = (cur_pos - kf_pos_).norm() + 0.1*(cur_quat - kf_quat_).norm();

	last_pose_ = pose;

	bool new_keyframe = false;
	if(dist_from_kf > new_kf_dist_thres_)
	{
		new_keyframe = true;
		kf_pos_ = cur_pos;
		kf_quat_ << cur_quat;
	}
	return new_keyframe;
}


cv::Mat EmvsNode::undistortImage(const cv::Mat input_image)
{
	cv::Mat output_image(sensor_rows, sensor_cols, EVENT_IMAGE_TYPE, cv::Scalar::all(0));
	cv::undistort(input_image, output_image, K_camera, D_camera);
	return output_image;
}

void EmvsNode::updateDsi(Mat event_img)
{
	// TODO break this up into smaller functions

	// Find transform between current pose and keyframe (use quat2rotm and Eigen)
	// find world->kf, world->image. then do w->kf * inv(w->img) (4x4 matrices)
	Mat kf_M = makeTransformMatrix(kf_pos_[0], kf_pos_[1], kf_pos_[2],
								   kf_quat_[0], kf_quat_[1], kf_quat_[2], kf_quat_[3]);

	Mat img_M = makeTransformMatrix(last_pose_.pose.position.x,
									last_pose_.pose.position.y,
									last_pose_.pose.position.z,
									last_pose_.pose.orientation.x,
									last_pose_.pose.orientation.y,
									last_pose_.pose.orientation.z,
									last_pose_.pose.orientation.w);

	Mat T_i2kf = kf_M * img_M.inv();

	Mat t(3, 1, DOUBLE_TYPE);
	T_i2kf.col(3).rowRange(0,3).copyTo(t);
	Mat n = (Mat_<double>(3,1) << 0, 0, -1);

	// precompute some reused matrices. using name from matlab code here - TODO clarify var names
	Mat R_transpose(3, 3, DOUBLE_TYPE); //
	T_i2kf.rowRange(0,3).colRange(0,3).copyTo(R_transpose);
	Mat R_t_n = R_transpose*t*n.t();

	// TODO For each plane, compute homography from image to plane, warp event image to plane and add
	Mat H_i2z;
	// std::cout << "Showing dsi" << std::endl;
	for(int i=0; i<kf_dsi_.N_planes_; i++)
	{
		double depth = kf_dsi_.planes_depths_[i];

		H_i2z = (K_camera * (R_transpose + R_t_n/depth)*K_camera.inv()).inv();
		// std::cout << "H_i2z: " << H_i2z << "\n";

		cv::Mat event_img_warped;
		cv::warpPerspective(event_img, event_img_warped, H_i2z, kf_dsi_.dsi_[i].size());

		kf_dsi_.dsi_[i] += event_img_warped;
	}
}

void EmvsNode::addDsiToMap()
{
	PointCloud new_points_kf_frame = kf_dsi_.getFiltered3dPoints();

	// TODO transform points to world frame
	PointCloud new_points_world_frame;

	//Add points to pointcloud, publish
	// map_points_ += new_points;
}

} // end namespace emvs

int main(int argc, char **argv)
{
	ros::init(argc, argv, "emvs_node");
	emvs::EmvsNode emvs_node;
	ros::spin();

	return 0;
}
