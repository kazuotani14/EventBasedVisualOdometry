#ifndef _KF_DSI_H_
#define _KF_DSI_H_

#include <vector>
#include <math.h>

#include <cv.h>
#include <opencv2/opencv.hpp>

#include "matlab_utils.h"
#include "filters.h"

class KeyframeDSI
{
public:
	KeyframeDSI(double im_height, double im_width, double min_depth, double max_depth,
				int N_planes, double fx, double fy);
	void resetDSI();
	cv::Mat getDepthmap();

private:
	double min_depth_, max_depth_; //[m]
	double im_height_, im_width_;
	double fx_, fy_; //[pixels]
	int N_planes_;

	std::vector<std::vector<double> > planes_scaling_; //scaling of size of each layer (in world units) wrt to pixels
	std::vector<double> planes_depths_; //depths of each layer [m]
	std::vector<cv::Mat> dsi_; //data structure holding voxel grid values of disparity space image

};

#endif
