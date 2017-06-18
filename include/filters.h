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

#include "utilities.h"

namespace emvs{

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void radiusFilter(PointCloud cloud, double search_radius=0.8, int min_neighbors=2);
void findMaxVals3D(const std::vector<cv::Mat>& images, cv::Mat& output);

} // end namespace emvs

#endif
