// Test gaussian blur and median filter, taking max across multiple images

#include <vector>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/package.h>

static const std::string OPENCV_WINDOW = "Image";

void maxValLoc3dSimple(const std::vector<cv::Mat>& images, cv::Mat& max_vals, cv::Mat& max_locs)
{
	assert(images[0].rows == max_vals.rows);
	assert(images[0].cols == max_vals.cols);

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

int main(int argc, char** argv)
{
	cv::namedWindow(OPENCV_WINDOW);
	std::string path = ros::package::getPath("evo");

	std::cout << path << std::endl;
	cv::Mat src, gauss_filtered, median_filtered;
	src = cv::imread(path+"/data/polka_dots.jpg", 0);
	gauss_filtered = src.clone();
	median_filtered = src.clone();

	// Trying different filters
	// cv::GaussianBlur(src, gauss_filtered, cv::Size(15, 15), 0, 0 ); //set sigmas to zero have them calculated based on kernel size
	// cv::medianBlur(src, median_filtered, 15); //window size must be odd
	//
	// cv::imshow(OPENCV_WINDOW, src);
	// cv::waitKey(0);
	// cv::imshow(OPENCV_WINDOW, gauss_filtered);
	// cv::waitKey(0);
	// cv::imshow(OPENCV_WINDOW, median_filtered);
	// cv::waitKey(0);

	// Trying max across multiple images
	int n_images = 10;
	std::vector<cv::Mat> stack_of_images(n_images);
	// 1. generate multiple images by blurring
	int filter_size = 3;
	int image_h = src.rows;
	int image_w = src.cols;
	for(int i=0; i<n_images; i++)
	{
		cv::GaussianBlur(src, gauss_filtered, cv::Size(filter_size, filter_size), 0, 0 );
		stack_of_images[i] = gauss_filtered;
		filter_size += 6;

		// stack_of_images[i] = cv::Mat(image_h, image_w, CV_16SC1, cv::Scalar(i));

		// cv::imshow(OPENCV_WINDOW, stack_of_images[i]);
		// cv::waitKey(500);
	}

	//TODO figure out better way to do this
	// 2. Find max across all of the images, and their location
	cv::Mat max_vals, max_locs;
	max_vals = cv::Mat(image_h, image_w, CV_16SC1, cv::Scalar(0));
	max_locs = max_vals.clone();

	maxValLoc3dSimple(stack_of_images, max_vals, max_locs);

	// std::cout << "max_vals:\n" << max_vals << "\n\n";
	// std::cout << "max_locs:\n" << max_locs << "\n\n";

	// cv::imshow(OPENCV_WINDOW, max_vals);
	// cv::waitKey(1);

	return 0;
}
