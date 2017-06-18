#ifndef _KF_DSI_H_
#define _KF_DSI_H_

#include <vector>
#include <math.h>
#include <iostream>

#include <cv.h>
#include <opencv2/opencv.hpp>

#include "filters.h"
#include "utilities.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace emvs{

class KeyframeDSI
{
public:
	KeyframeDSI(double im_height, double im_width, double min_depth, double max_depth,
				int N_planes, double fx, double fy);
	void resetDSI();

	PointCloud getFiltered3dPoints();

	std::vector<std::vector<double> > planes_scaling_; //scaling of size of each layer (in world units) wrt to pixels
	std::vector<double> planes_depths_; //depths of each layer [m]
	std::vector<cv::Mat> dsi_; //data structure holding voxel grid values of disparity space image
	int N_planes_;

private:
	double min_depth_, max_depth_; //[m]
	double im_height_, im_width_;
	double fx_, fy_; //[pixels]

	// Intermediate steps for getting filtered 3d points in local frame
	void getDepthmap(cv::Mat& output);
	void projectDepthmapTo3d(PointCloud& output);

};

} // end namespace emvs

#endif
