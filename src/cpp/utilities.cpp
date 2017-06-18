#include "utilities.h"

namespace emvs{

void showNormalizedImage(const cv::Mat& image, int milliseconds)
{
	cv::Mat viz_event_image;
	cv::normalize(image, viz_event_image, 0, 255, cv::NORM_MINMAX);
	cv::imshow(OPENCV_WINDOW, viz_event_image);
	cv::waitKey(milliseconds);
}

cv::Mat quat2rotm(double q_x, double q_y, double q_z, double q_w)
{
	cv::Mat rotm = cv::Mat::zeros(3, 3, DOUBLE_TYPE);

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

cv::Mat quat2rotm(const Eigen::Vector4d q)
{
	return quat2rotm(q[0], q[1], q[2], q[3]);
}

cv::Mat makeTransformMatrix(double x, double y, double z, double qx, double qy, double qz, double qw)
{
	cv::Mat M(4, 4, DOUBLE_TYPE);
	cv::Mat T = (cv::Mat_<double>(3,1) << x, y, z);
	cv::Mat R = quat2rotm(qx, qy, qz, qw);
	cv::Mat zeros_one = (cv::Mat_<double>(1,4) << 0, 0, 0, 1);
	R.copyTo(M.colRange(0,3).rowRange(0,3));
	T.copyTo(M.col(3).rowRange(0,3));
	zeros_one.copyTo(M.row(3));
	return M;
}

}
