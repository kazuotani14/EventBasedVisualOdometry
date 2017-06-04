#include "emvs_node.h"

EmvsNode::EmvsNode()
	: new_pose_estimate_available_(false),
	  new_keyframe_(true),
	  kf_dsi_(sensor_rows, sensor_cols, min_depth, max_depth, N_planes, fx, fy)
{
	events_sub_ = nh_.subscribe("dvs/events", 1000, &EmvsNode::eventCallback, this);
	ground_truth_sub_ = nh_.subscribe("optitrack/davis", 100, &EmvsNode::poseCallback, this);
	// camera_info_sub_ = nh_.subscribe("dvs/camera_info", 1, &EmvsNode::camerainfoCallback, this);

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

void EmvsNode::updateDSI()
{
	// Output image
	// cv::Mat im_out;
	// Warp source image to destination based on homography
	// cv::warpPerspective(im_src, im_out, h, im_dst.size());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "emvs_node");
	EmvsNode emvs_node;
	ros::spin();

	return 0;
}
