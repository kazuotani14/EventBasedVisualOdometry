#ifndef _FILTERS_H_
#define _FILTERS_H_

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

//OpenCV
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <algorithm>
#include "utilities.h"

namespace emvs{

// Given a 3D matrix (stack of images), find the locations and values of the pixel-wise maximums
void findMaxVals3D(const std::vector<cv::Mat>& images, cv::Mat& max_layers, cv::Mat& max_vals);

// Median filter over non-zero values of image
cv::Mat medianFilterNonZero(cv::Mat& img, int radius);

// Removes outlier points from pointcloud - check for >threshold # of neighbors
PointCloud radiusFilter(PointCloud& cloud, double search_radius=0.8, int min_neighbors=2);

} // end namespace emvs

#endif
