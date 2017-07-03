#include "filters.h"

namespace emvs{

// TODO find a better/faster way to do this
// Use vectorization in Eigen, MKL backend (free for students)
// Figure out how to convert between Eigen matrices and cv::Mat cheaply
// Turn on compiler flags - https://github.com/resibots/limbo/blob/master/wscript
// 	native_flags from link above
void findMaxVals3D(const std::vector<cv::Mat>& images, cv::Mat& max_layers, cv::Mat& max_vals)
{
	int im_height = images[0].rows;
	int im_width = images[0].cols;

	for(int i=0; i<im_height; i++){
		for(int j=0; j<im_width; j++){
			for(int z=0; z<images.size(); z++){
				if(static_cast<int>(images[z].at<uchar>(i,j)) > static_cast<int>(max_vals.at<uchar>(i,j))){
					max_layers.at<uchar>(i,j) = z;
					max_vals.at<uchar>(i,j) = static_cast<int>(images[z].at<uchar>(i,j));
				}
			}
		}
	}
}

// TODO make this better... there has to be some opencv trick for this
// TODO consider zero-padding
cv::Mat medianFilterNonZero(cv::Mat& img, int radius)
{
	std::vector<uchar> vals(9);

	cv::Mat filtered = cv::Mat::zeros(img.rows, img.cols, EVENT_IMAGE_TYPE);

	for(int x=radius; x<(img.rows-radius); x++){
		for(int y=radius; y<(img.cols-radius); y++){
			if(img.at<uchar>(x,y) == 0) continue;

			//extract values from block
			int val;
			for(int i=-radius; i<radius; i++){
				for(int j=-radius; j<radius; j++){
					val = img.at<uchar>(x+i, y+j);
					if(val==0) continue;
					vals.push_back(val);
				}
			}

			if(vals.empty()) continue;

			//find median of block
			std::nth_element(vals.begin(), vals.begin() + vals.size()/2, vals.end());
			int median = vals[vals.size()/2];

			//set value
			filtered.at<uchar>(x,y) = median;

			vals.clear();
		}
	}

	return filtered;
}

// TODO tune these parameters?
PointCloud radiusFilter(PointCloud& cloud, double search_radius, int min_neighbors)
{
	ROS_INFO("enter radius filter");
	PointCloud cloud_filtered;
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;

	PointCloud::Ptr cloud_ptr(&cloud);

	std::cout << "width: " << cloud_ptr->width << std::endl;

	outrem.setInputCloud(cloud_ptr);
	outrem.setRadiusSearch(search_radius);
	outrem.setMinNeighborsInRadius(min_neighbors);
	outrem.filter(cloud_filtered);

	return cloud_filtered;
}

} // end namespace emvs
