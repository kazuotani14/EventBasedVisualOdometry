#include "keyframe_dsi.h"

KeyframeDSI::KeyframeDSI(double im_height, double im_width, double min_depth, double max_depth,
						 int N_planes, double fx, double fy)
	: im_height_(im_height), im_width_(im_width),
	  min_depth_(min_depth), max_depth_(max_depth),
	  N_planes_(N_planes), fx_(fx), fy_(fy)
{
	std::cout << "init kf_dsi" << std::endl;
	// Reserve space for data
	planes_scaling_.reserve(N_planes_);
	planes_depths_.reserve(N_planes_);
	dsi_.reserve(N_planes_);

	// Find uniformly space depths
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

		dsi_[i] = cv::Mat::zeros(im_height_, im_width_, CV_16SC1);
	}

	resetDSI();
}

void KeyframeDSI::resetDSI()
{
	// Set zeros
	for (int i=0; i<N_planes_; i++)
		dsi_[i] = cv::Scalar(0);
}

cv::Mat KeyframeDSI::getDepthmap()
{
	int gauss_filter_size = 5;
	int median_filter_size = 15; //window size must be odd

	// 1. Gaussian filter on each layer
	cv::Mat gauss_filtered = cv::Mat(im_height_, im_width_, CV_16SC1, cv::Scalar(0));
	for(int i=0; i<N_planes_; i++)
	{
		cv::GaussianBlur(dsi_[i], gauss_filtered, cv::Size(gauss_filter_size, gauss_filter_size), 0, 0);
		dsi_[i] = gauss_filtered;
		// cv::imshow(OPENCV_WINDOW, stack_of_images[i]);
		// cv::waitKey(500);
	}

	// 2. Find max across all of the images, and their location
	//TODO figure out better way to do this
	cv::Mat max_vals, max_locs;
	findMaxVals3D(dsi_, max_vals, max_locs);

	// TODO 3. threshold on max to get intermediate depthmap
	cv::Mat interm_depth_map = cv::Mat(im_height_, im_width_, CV_16SC1, cv::Scalar(0));
	//interm_depth_map = confidence_map > (imfilter(confidence_map, filter) - C); // C=-2

	// 4. Median filter on depth map
	cv::Mat final_depth_map = cv::Mat(im_height_, im_width_, CV_16SC1, cv::Scalar(0));
	cv::medianBlur(interm_depth_map, final_depth_map, median_filter_size);
	// TODO matlab code finds median over non-zero values - may have to implement custom

	return final_depth_map;
}
