#ifndef _OPENCV_DEFS_H_
#define _OPENCV_DEFS_H_

#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <Eigen/Core>

#include <cv.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Commonly used OpenCV Mat types
#define EVENT_IMAGE_TYPE CV_8U
#define DOUBLE_TYPE CV_64F

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace emvs{

// Visualize opencv image, with normalized values so that low values are visible
void showNormalizedImage(std::string window, const cv::Mat& image, int milliseconds=1);

// Quaternion to rotation matrix
cv::Mat quat2rotm(double q_x, double q_y, double q_z, double q_w);
cv::Mat quat2rotm(const Eigen::Vector4d q);

// Make 4x4 homogeneous transformation matrix from relative position and rotation values
cv::Mat makeTransformMatrix(double x, double y, double z, double qx, double qy, double qz, double qw);


// C++ implementation of Matlab linspace
// From: https://gist.github.com/jmbr/2375233
template <typename T>
std::vector<T> linspace(T start, T end, int N)
{
	std::vector<T> vec(N);

	T h = (end - start) / static_cast<T>(N-1);

	vec[0] = start;
	for(int i=1; i<N; i++)
		vec[i] = vec[i-1] + h;

	return vec;
}

} // end namespace emvs

#endif
