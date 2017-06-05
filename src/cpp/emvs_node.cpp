#include "emvs_node.h"

EmvsNode::EmvsNode()
	: new_pose_estimate_available_(false),
	  new_keyframe_(true),
	  kf_dsi_(sensor_rows, sensor_cols, min_depth, max_depth, N_planes, fx, fy)
{
	events_sub_ = nh_.subscribe("dvs/events", 1000, &EmvsNode::eventCallback, this);
	ground_truth_sub_ = nh_.subscribe("optitrack/davis", 100, &EmvsNode::poseCallback, this);
	// camera_info_sub_ = nh_.subscribe("dvs/camera_info", 1, &EmvsNode::camerainfoCallback, this);

	pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("map_points", 1);
	map_points_.height = 1;

	latest_event_image_ = cv::Mat::zeros(sensor_rows, sensor_cols, CV_16SC1);
	new_event_image_ = cv::Mat::zeros(sensor_rows, sensor_cols, CV_16SC1);

	cv::namedWindow(OPENCV_WINDOW);
	ROS_INFO("emvs node init done");

}

EmvsNode::~EmvsNode()
{
	cv::destroyWindow(OPENCV_WINDOW);
}

void EmvsNode::eventCallback(const dvs_msgs::EventArray& msg)
{
	//TODO Make better way to get event image rather than just waiting for state estimate

	// ROS_INFO("Events[i]: %d, %d", static_cast<int>(msg.events[i].y),  static_cast<int>(msg.events[i].x));
	// ROS_INFO("EventArray size: %d", static_cast<int>(msg.events.size()));

	// for(auto const event&: msg.events)
	for(int i=0; i<msg.events.size(); i++)
	{
		new_event_image_.at<short>(static_cast<int>(msg.events[i].y), static_cast<int>(msg.events[i].x)) += 5000;
	}
}

void EmvsNode::poseCallback(const geometry_msgs::PoseStamped& msg)
{
	// ROS_INFO("received pose estimate");

	//TODO check for new keyframe (dist threshold)

	if(new_keyframe_)
	{
		// TODO add current DSI to map, if not first time

		kf_dsi_.resetDSI();
		new_keyframe_ = false;
	}
	else
	{
		latest_pose_estimate_ = msg;
		new_pose_estimate_available_ = true;
		new_event_image_.copyTo(latest_event_image_);
		new_event_image_ = cv::Scalar(0);

		// Undistort image
		latest_event_image_ = undistortImage(latest_event_image_);

		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, latest_event_image_);
		cv::waitKey(100);
		// TODO Find out why event images aren't shown in the first few secs

		//TODO send latest_event_image_(undisto) to updatedsi
	}

}

cv::Mat EmvsNode::undistortImage(const cv::Mat& input_image)
{
	// TODO check if this function needs

	cv::Mat output_image(sensor_rows, sensor_cols, CV_16SC1, cv::Scalar::all(0));
	cv::undistort(input_image, output_image, K_camera, D_camera);
	return output_image;
}

void EmvsNode::updateDsi()
{
	// TODO break this up into smaller functions

	// TODO Find transform between current pose and keyframe (use quat2rotm and Eigen)
	// find world->kf, world->image. then do w->kf * inv(w->img) (3x4 matrices)
	// kf_T = kf_pose(2:4)';
	// kf_quat = kf_pose(5:8);
	// kf_R = CustomQuat2RotM(kf_quat);
	// kf_M = [kf_R, kf_T; 0 0 0 1];
	//
	// i_T = i_pose(2:4)';
	// i_quat = i_pose(5:8);
	// i_R = CustomQuat2RotM(i_quat);
	// i_M = [i_R, i_T; 0 0 0 1];

	//TODO precompute some reused matrices
	// R_transpose = T_i_in_kf(1:3,1:3)';
	// R_t_n = R_transpose*t*n';

	// TODO for each plane, compute homography from image to planes
	// Warp image and add to kf_dsi_ (should kf_dsi_ be eigen matrices?)
	// cv::Mat im_out;
	// Warp source image to destination based on homography
	// cv::warpPerspective(im_src, im_out, h, im_dst.size());
}

void EmvsNode::addDsiToMap()
{
	//TODO make these functions!
	//getDepthmap: gaussian blur on each layer, then take max from each layer to return wxh depth map (opencv)
	cv::Mat depth_map = kf_dsi_.getDepthmap();

	//ProjectDsiPointsTo3d: get 3d point coordinates from filtered depth map (manually?)
	PointCloud new_points;

	//RadiusFilter: radius outlier removal of resulting (use pcl)


	//Add points to pointcloud, publish
	map_points_ += new_points;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "emvs_node");
	EmvsNode emvs_node;
	ros::spin();

	return 0;
}
