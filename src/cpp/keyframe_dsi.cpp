#include "keyframe_dsi.h"

namespace emvs{

KeyframeDsi::KeyframeDsi(double im_height, double im_width, double min_depth, double max_depth,
						 int N_planes, double fx, double fy)
	: im_height_(im_height), im_width_(im_width),
	  min_depth_(min_depth), max_depth_(max_depth),
	  N_planes_(N_planes), fx_(fx), fy_(fy)
{
	// Reserve space for data
	planes_scaling_.resize(N_planes_);
	planes_depths_.resize(N_planes_);
	dsi_.resize(N_planes_);

	// Find uniformly spaced planes, either in depth or inverse depth
	// *Original EMVS used depth - "we used depth instead of inverse depth in the
	// DSI since it provided better results in scenes with finite depth variations"
	// *EVO used inverse depth - "allow for mapping far away objects, we discretize
	// the DSI volume using depth planes uniformly spaced in inverse depth"
	// *EMVS used N=100, EVO used N=50

	// Uniform depth
	planes_depths_ = linspace(min_depth_, max_depth_, N_planes_);

	// Uniform inverse depth
	// std::vector<double> planes_depths = linspace(1/min_depth_, 1/max_depth_, N_planes_);
	// std::for_each(planes_depths.begin(), planes_depths.end(), [](double& d){ d = 1/d;});

	// Find scaling to real world units
	double cone_angle_x = atan((im_width/2) / fx_); //half of FOV
	double cone_angle_y = atan((im_height/2) / fy_); //half of FOV

	for(int i=0; i<N_planes_; i++)
	{
		double depth = planes_depths_[i];
		double frustumHeight = 2 * depth * tan(cone_angle_y);
		double frustumWidth = 2 * depth * tan(cone_angle_x);

		//"pixels" to meters (i.e. d_pixels * scale = d_world)
		double scale_x = frustumHeight / im_height;
		double scale_y = frustumWidth / im_width;
		planes_scaling_[i] = {scale_x, scale_y};

		dsi_[i] = cv::Mat::zeros(im_height_, im_width_, EVENT_IMAGE_TYPE);
	}
}

void KeyframeDsi::resetDsi()
{
	// Set zeros
	for (int i=0; i<N_planes_; i++)
		dsi_[i].setTo(0);
}

KeyframeDsi* KeyframeDsi::clone() const
{
	KeyframeDsi* clone_dsi = new KeyframeDsi(im_height_, im_width_, min_depth_, max_depth_, N_planes_,  fx_,  fy_);

	// add to zeros instead of copying - just cuz it's more convenient than trying to copy private member
	for(int i=0; i<N_planes_; i++)
	{
		clone_dsi->addToDsi(dsi_[i], i);
	}

	return clone_dsi;
}

int KeyframeDsi::getPlaneDepth(const int layer)
{
	return planes_depths_[layer];
}

void KeyframeDsi::addToDsi(const cv::Mat& events, const int layer)
{
	dsi_[layer] += events;
}

// Returns filtered set of 3D points from DSI, in keyframe frame.
PointCloud KeyframeDsi::getFiltered3dPoints()
{
	//getDepthmap: gaussian blur on each layer, then take max from each layer to return wxh depth map (opencv)
	cv::Mat depthmap = cv::Mat(im_height_, im_width_, EVENT_IMAGE_TYPE, cv::Scalar(0));
	getDepthmap(depthmap);

	// TODO verify with fake depthmap
	//ProjectDsiPointsTo3d: get 3d point coordinates from filtered depth map
	PointCloud new_points;
	new_points.height = cv::countNonZero(depthmap);
	new_points.width = 1;
	new_points.points.resize(cv::countNonZero(depthmap));
	projectDepthmapTo3d(depthmap, new_points);

	// TODO fix and enable this
	//RadiusFilter: radius outlier removal of resulting (use pcl)
	// if (cv::countNonZero(depthmap) > 0)
	// {
	// 	PointCloud filtered_new_points;
	// 	filtered_new_points = radiusFilter(new_points);
	// 	return filtered_new_points;
	// }
	// else{
	// 	return new_points;
	// }
	return new_points;
}


void KeyframeDsi::getDepthmap(cv::Mat& output)
{
	// 1. Find max across all of the images, and their location
	//TODO figure out better way to implement findMaxVals3D - linear search right now
	cv::Mat max_depths = cv::Mat(im_height_, im_width_, EVENT_IMAGE_TYPE, cv::Scalar(0));
	cv::Mat max_vals = cv::Mat(im_height_, im_width_, EVENT_IMAGE_TYPE, cv::Scalar(0));
	findMaxVals3D(dsi_, max_depths, max_vals);

	// 2. Gaussian filter on confidence map
	cv::Mat filtered(im_height_, im_width_, EVENT_IMAGE_TYPE, cv::Scalar(0));
	cv::GaussianBlur(max_vals, filtered, cv::Size(gauss_filter_size, gauss_filter_size), guass_filter_sigma);

	// 3. threshold on filtered to get intermediate depthmap
	// values should come from max_locs
	cv::Mat thresholded;
	cv::compare(max_vals, (filtered + confidence_map_cushion), thresholded, cv::CMP_GT);
	thresholded /= 255;

	cv::Mat depthmap;
	cv::multiply(thresholded, max_depths, depthmap);

	// 4. TODO Median filter on *non-zero values* of depth map
	// cv::Mat final_depthmap;
	// cv::medianBlur(depthmap, final_depthmap, median_filter_size);

	output = depthmap;
}

void KeyframeDsi::projectDepthmapTo3d(cv::Mat& depthmap, PointCloud& points_camera_frame)
{
	std::vector<cv::Point> nonzero_locations;
	cv::findNonZero(depthmap, nonzero_locations);
	points_camera_frame.height = nonzero_locations.size();
	points_camera_frame.width = 1;
	points_camera_frame.points.resize(points_camera_frame.height * points_camera_frame.width);

	for(int i=0; i<nonzero_locations.size(); i++)
	{
		double layer = depthmap.at<double>(nonzero_locations[i].y, nonzero_locations[i].x);
		points_camera_frame.points[i].x = (nonzero_locations[i].x - im_width_/2) * planes_scaling_[layer][0];
		points_camera_frame.points[i].y = (nonzero_locations[i].y - im_height_	/2) * planes_scaling_[layer][1];
		points_camera_frame.points[i].z = planes_depths_[layer];
	}
}

} // end namespace emvs
