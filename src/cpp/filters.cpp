#include "filters.h"

void radiusFilter(PointCloud cloud, double search_radius, int min_neighbors) // TODO tune these params
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	PointCloud::Ptr cloud_ptr(&cloud);
	PointCloud::Ptr cloud_filtered(new PointCloud);

	outrem.setInputCloud(cloud_ptr);
	outrem.setRadiusSearch(search_radius);
	outrem.setMinNeighborsInRadius(min_neighbors);
	outrem.filter(*cloud_filtered);

	cloud = *cloud_filtered; // TODO make sure this is ok
}

// TODO find a better/faster way to do this
void findMaxVals3D(const std::vector<cv::Mat>& images, cv::Mat& max_vals, cv::Mat& max_locs)
{
	int im_height = images[0].rows;
	int im_width = images[0].cols;
	max_vals = cv::Mat(im_height, im_width, CV_16SC1, cv::Scalar(0));
	max_locs = max_vals.clone();

	for(int i=0; i<images[0].rows; i++){
		for(int j=0; j<images[0].cols; j++){
			for(int z=0; z<images.size(); z++){
				if(images[z].at<short>(i,j) > max_vals.at<short>(i,j)){
					max_vals.at<short>(i,j) = images[z].at<short>(i,j);
					max_locs.at<short>(i,j) = z;
				}
			}
		}
	}
}
