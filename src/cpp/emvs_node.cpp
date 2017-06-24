#include "emvs_node.h"

namespace emvs{


EmvsNode::EmvsNode()
	: kf_dsi_(sensor_rows, sensor_cols, min_depth, max_depth, N_planes, fx, fy)
{

	events_sub_ = nh_.subscribe("dvs/events", 1000, &EmvsNode::eventCallback, this);
	ground_truth_sub_ = nh_.subscribe("optitrack/davis", 100, &EmvsNode::poseCallback, this);
	// camera_info_sub_ = nh_.subscribe("dvs/camera_info", 1, &EmvsNode::camerainfoCallback, this);

	pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("map_points", 1);
	map_points_.height = 1;

	new_events_ = Mat::zeros(sensor_rows, sensor_cols, EVENT_IMAGE_TYPE);

	cv::namedWindow(OPENCV_WINDOW);

	events_to_dsi_th_ = std::thread(std::bind(&EmvsNode::process_events_to_dsi, this));
	dsi_to_map_th_ = std::thread(std::bind(&EmvsNode::process_dsi_to_map, this));
}

EmvsNode::~EmvsNode()
{
	cv::destroyWindow(OPENCV_WINDOW);
}

// Add events to DSI
// Events come in batches at 30Hz, while pose estimates come in at 200Hz
void EmvsNode::eventCallback(const dvs_msgs::EventArray& msg)
{
	// ROS_INFO("Received event set at %f from %d to %d", msg.header.stamp.toSec(), msg.events[0].ts, msg.events[n-1].ts);

	// TODO Make this better
	// For now, approximate fast "event image" rate by dividing each set of events into smaller subsets
	// Save poses received up until now into a queue. Ideally, we would look at timestamps and assign accordingly
	int n_events = msg.events.size();
	int n_poses = received_poses_.size();
	if(!n_poses) return;
	int events_per_pose = ceil(n_events/n_poses);

	ROS_INFO("n_events: %d, n_poses: %d", n_events, n_poses);

	for(int i=0; i<n_poses; i++)
	{
		for(int j=i*events_per_pose; j<(i+1)*events_per_pose || j<n_events; j++)
		{
			new_events_.at<uchar>(static_cast<int>(msg.events[j].y), static_cast<int>(msg.events[j].x)) += 1;
		}

		Mat undistorted_events_ = undistortImage(new_events_);
		events_to_dsi_queue_.push(std::make_pair(undistorted_events_, received_poses_.front()));
		received_poses_.pop();

		showNormalizedImage(undistorted_events_);
		new_events_.setTo(0);
	}
}

void EmvsNode::poseCallback(const geometry_msgs::PoseStamped& msg)
{
	// ROS_INFO("Received pose at %f", msg.header.stamp.toSec());

	bool new_kf = checkForNewKeyframe(msg);

	if(new_kf)
	{
		std::shared_ptr<KeyframeDsi> ptr(kf_dsi_.clone());
		dsi_to_map_queue_.push(ptr);  // TODO think more about this - is copying the most efficient?
		kf_dsi_.resetDsi();

		kf_pos_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
		kf_quat_ << msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w;
	}
	else
	{
    	received_poses_.push(msg);
	}

	// publish tf for visualization
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "davis_camera";
	transformStamped.transform.translation.x = msg.pose.position.x;
	transformStamped.transform.translation.y = msg.pose.position.y;
	transformStamped.transform.translation.z = msg.pose.position.z;
	transformStamped.transform.rotation = msg.pose.orientation;
	tf_br_.sendTransform(transformStamped);
}

void EmvsNode::process_events_to_dsi()
{
	while(ros::ok())
	{
		std::lock_guard<std::mutex> lock(dsi_mutex_);
		if(!events_to_dsi_queue_.empty())
		{
			auto pair = events_to_dsi_queue_.front(); // TODO get pointer instead?
			addEventsToDsi(pair.first, pair.second);
			events_to_dsi_queue_.pop();
		}
		// ROS_INFO("hey!");
	}
}

void EmvsNode::process_dsi_to_map()
{
	while(ros::ok())
	{
		if(!dsi_to_map_queue_.empty())
		{
			std::shared_ptr<KeyframeDsi> kf_ptr = dsi_to_map_queue_.front();
			addDsiToMap(*kf_ptr); // TODO put this on separate queue. make a new dsi? or get depthmap and copy that?
			dsi_to_map_queue_.pop();
		}
	}
}

// Check input pose against current keyframe pose (currently position dist threshold)
bool EmvsNode::checkForNewKeyframe(const geometry_msgs::PoseStamped& pose)
{
	Eigen::Vector3d cur_pos;
	cur_pos << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
	Eigen::Vector4d cur_quat;
	cur_quat << pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w;
	double dist_from_kf = (cur_pos - kf_pos_).norm() + 0.1*(cur_quat - kf_quat_).norm(); // arbitrarily weigh rotations down

	bool new_keyframe = (dist_from_kf > new_kf_dist_thres_) ? true : false;
	return new_keyframe;
}

Mat EmvsNode::undistortImage(const Mat input_image)
{
	cv::Mat output_image(sensor_rows, sensor_cols, EVENT_IMAGE_TYPE, cv::Scalar::all(0));
	cv::undistort(input_image, output_image, K_camera, D_camera);
	return output_image;
}

void EmvsNode::addEventsToDsi(const Mat& events, const geometry_msgs::PoseStamped& cam_pose)
{
	// Equations from Gallup, David, et al.
	//"Real-time plane-sweeping stereo with multiple sweeping directions." CVPR 2007

	// Find transform between current pose and keyframe
	// Find world->kf, world->camera. then do world->kf * inv(world->camera)
	Mat kf_M = makeTransformMatrix(kf_pos_[0], kf_pos_[1], kf_pos_[2],
								   kf_quat_[0], kf_quat_[1], kf_quat_[2], kf_quat_[3]);

	Mat cam_M = makeTransformMatrix(cam_pose.pose.position.x,
									cam_pose.pose.position.y,
									cam_pose.pose.position.z,
									cam_pose.pose.orientation.x,
									cam_pose.pose.orientation.y,
									cam_pose.pose.orientation.z,
									cam_pose.pose.orientation.w);

	Mat T_c2kf = kf_M * cam_M.inv();

	// Precompute some reused matrices
	Mat t(3, 1, DOUBLE_TYPE);
	T_c2kf.col(3).rowRange(0,3).copyTo(t);
	Mat n = (Mat_<double>(3,1) << 0, 0, -1);

	Mat R_transpose(3, 3, DOUBLE_TYPE);
	T_c2kf.rowRange(0,3).colRange(0,3).copyTo(R_transpose);
	Mat R_t_n = R_transpose*t*n.t();

	// For each plane, compute homography from image to plane, warp event image to plane and add to DSI
	Mat H_c2z;
	for(int i=0; i<kf_dsi_.N_planes_; i++)
	{
		double depth = kf_dsi_.getPlaneDepth(i);
		H_c2z = (K_camera * (R_transpose + R_t_n/depth)*K_camera.inv()).inv();

		cv::Mat event_img_warped;
		cv::warpPerspective(events, event_img_warped, H_c2z, cv::Size(sensor_cols, sensor_rows));

		kf_dsi_.addToDsi(event_img_warped, i);
	}
}

void EmvsNode::addDsiToMap(KeyframeDsi& kf_dsi)
{
	PointCloud new_points_kf_frame = kf_dsi.getFiltered3dPoints();

	// Transform points to world frame
	Mat world_to_kf_mat = makeTransformMatrix(kf_pos_[0], kf_pos_[1], kf_pos_[2],
							kf_quat_[0], kf_quat_[1], kf_quat_[2],kf_quat_[3]);
	Eigen::Matrix4f world_to_kf;
	cv2eigen(world_to_kf_mat, world_to_kf);
	PointCloud new_points_world_frame;
	pcl::transformPointCloud(new_points_kf_frame, new_points_world_frame, world_to_kf);

	map_points_ += new_points_world_frame;

	// Publish map pointcloud
	sensor_msgs::PointCloud2 msg;
	pcl::toROSMsg(map_points_, msg);
	msg.header.frame_id = "world";
	pointcloud_pub_.publish(msg);
}

} // end namespace emvs

int main(int argc, char **argv)
{
	ros::init(argc, argv, "emvs_node");
	emvs::EmvsNode emvs_node;
	ros::spin();

	return 0;
}
