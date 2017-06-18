#ifndef _IMAGE_CONVERVTER_H_
#define _IMAGE_CONVERVTER_H_

// From http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include "opencv_defs.h"

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber events_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter();
  ~ImageConverter();

  cv::Mat new_event_image_;
  void imageCb(const dvs_msgs::EventArray& msg);
};

#endif
