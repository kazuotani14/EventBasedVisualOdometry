// C++ implementations of matlab functions (helpers)
// TODO move this stuff into its own namespace?

#ifndef _MATLAB_UTILS_H_
#define _MATLAB_UTILS_H_

#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "opencv_defs.h"

namespace emvs{

// From: https://gist.github.com/jmbr/2375233
// TODO look at link for possible improvements
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

cv::Mat quat2rotm(double q_x, double q_y, double q_z, double q_w);

cv::Mat quat2rotm(const Eigen::Vector4d q);

} // end namespace emvs

#endif
