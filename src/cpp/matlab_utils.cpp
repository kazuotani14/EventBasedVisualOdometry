#include "matlab_utils.h"

cv::Mat quat2rotm(double q_x, double q_y, double q_z, double q_w)
{
	cv::Mat rotm = cv::Mat::zeros(3, 3, CV_64FC1);

	rotm.at<double>(0,0) = 1 - 2*pow(q_y,2) - 2*pow(q_z,2);
	rotm.at<double>(0,1) = 2*q_x*q_y - 2*q_z*q_w;
	rotm.at<double>(0,2) = 2*q_x*q_z + 2*q_y*q_w;
	rotm.at<double>(1,0) = 2*q_x*q_y + 2*q_z*q_w;
	rotm.at<double>(1,1) = 1 - 2*pow(q_x,2) - 2*pow(q_z,2);
	rotm.at<double>(1,2) = 2*q_y*q_z - 2*q_x*q_w;
	rotm.at<double>(2,0) = 2*q_x*q_z - 2*q_y*q_w;
	rotm.at<double>(2,1) = 2*q_y*q_z + 2*q_x*q_w;
	rotm.at<double>(2,2) = 1 - 2*pow(q_x,2) - 2*pow(q_y,2);

	return rotm;
}
