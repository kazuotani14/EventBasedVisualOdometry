#include "filters.h"

namespace emvs{

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
void findMaxVals3D(const std::vector<cv::Mat>& images, cv::Mat& max_locs)
{
	int im_height = images[0].rows;
	int im_width = images[0].cols;
	Eigen::MatrixXd max_vals;
	max_vals = Eigen::MatrixXd::Zero(im_height,im_width);

	for(int i=0; i<im_height; i++){
		for(int j=0; j<im_width; j++){
			for(int z=0; z<images.size(); z++){
				if(static_cast<int>(images[z].at<uchar>(i,j)) > static_cast<int>(max_vals(i,j))){
					std::cout << static_cast<int>(images[z].at<uchar>(i,j)) << " : " << static_cast<int>(max_vals(i,j)) << std::endl;
					max_locs.at<uchar>(i,j) = z;
					max_vals(i,j) = static_cast<int>(images[z].at<uchar>(i,j));
					std::cout << static_cast<int>(max_vals(i,j)) << " z: " << z << '\n';
				}
				// std::cout << "z: " << z << std::endl;
			}
		}
	}

	std::cout << "max_locs: " << cv::countNonZero(max_locs) << std::endl;

}

} // end namespace emvs
