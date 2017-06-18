#include "image_converter.h"

ImageConverter::ImageConverter()
    : it_(nh_)
{
	// Subscrive to input video feed and publish output video feed
  events_sub_ = nh_.subscribe("dvs/events", 1000, &ImageConverter::imageCb, this);
	image_pub_ = it_.advertise("image_converter/output_video", 1);

  new_event_image_ = cv::Mat::zeros(180, 240, EVENT_IMAGE_TYPE);
	cv::namedWindow(OPENCV_WINDOW);
}

ImageConverter::~ImageConverter()
{
	cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::imageCb(const dvs_msgs::EventArray& msg)
{

  for(int i=0; i<msg.events.size(); i++)
  {
    // const short* Mi = new_event_image_.ptr<short>(static_cast<int>(msg.events[i].y));
    // Mi[static_cast<int>(msg.events[i].x)] += 5000;
    new_event_image_.at<uchar>(static_cast<int>(msg.events[i].y), static_cast<int>(msg.events[i].x)) += 5000;
    // new_event_image_(msg.events[i].y, msg.events[i].x) += 5000;
    // new_event_image_.at<short>(static_cast<int>(msg.events[i].y), static_cast<int>(msg.events[i].x)) += 5000; // TODO this shoud be 1, but is 5000 for viz purposes
    // std::cout << "raw: " << msg.events[i].y << " " << msg.events[i].x << std::endl;
    // std::cout << "cast: " << static_cast<int>(msg.events[i].y) << " " << static_cast<int>(msg.events[i].x) << std::endl;

  }

  sensor_msgs::ImagePtr img = cv_bridge::CvImage(std_msgs::Header(), "mono8", new_event_image_).toImageMsg();
  image_pub_.publish(img);

  cv::imshow(OPENCV_WINDOW, new_event_image_);
	cv::waitKey(3);
	//
  new_event_image_ = cv::Scalar(0);

	// cv_bridge::CvImagePtr cv_ptr;
	// try
	// {
	//   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	// }
	// catch (cv_bridge::Exception& e)
	// {
	//   ROS_ERROR("cv_bridge exception: %s", e.what());
	//   return;
	// }

	// // Draw an example circle on the video stream
	// if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	//   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
	//
	// // Update GUI Window
	// cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	// cv::waitKey(3);
	// //

	// Output modified video stream
	// image_pub_.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
