#include "filters.h"
#include "utilities.h"
#include <gtest/gtest.h>

TEST(FilterTests, maxvals)
{
  // findMaxVals3D(const std::vector<cv::Mat>& images, cv::Mat& max_layers, cv::Mat& max_vals)
  const int n_mat = 3;
  const int im_w = 3;
  const int im_h = 3;

  // Set up stack of matrices
  std::vector<cv::Mat> mat_stack;
  mat_stack.resize(n_mat);
  for(int i=0; i<n_mat; i++)
  {
    mat_stack[i] = cv::Mat::zeros(im_w, im_h, EVENT_IMAGE_TYPE);
  }

  // Add some values
  mat_stack[0].at<uchar>(0,0) = static_cast<int>(5);
  mat_stack[1].at<uchar>(0,0) = static_cast<int>(3);
  mat_stack[2].at<uchar>(0,0) = static_cast<int>(1);
  mat_stack[2].at<uchar>(1,1) = static_cast<int>(10);

  //Find maximum values for each pixel across depths
  cv::Mat max_layers = cv::Mat::zeros(im_w, im_h, EVENT_IMAGE_TYPE);
  cv::Mat max_vals = cv::Mat::zeros(im_w, im_h, EVENT_IMAGE_TYPE);
  emvs::findMaxVals3D(mat_stack, max_layers, max_vals);

  int max_val_00 = max_vals.at<uchar>(0,0);
  int max_layer_00 = max_layers.at<uchar>(0,0);
  int max_val_11 = max_vals.at<uchar>(1,1);
  int max_layer_11 = max_layers.at<uchar>(1,1);

  EXPECT_EQ(max_val_00, 5) << "max_vals(0,0) not correct";
  EXPECT_EQ(max_layer_00, 0) << "max_layers(0,0) not correct";
  EXPECT_EQ(max_val_11, 10) << "max_vals(1,1) not correct";
  EXPECT_EQ(max_layer_11, 2) << "max_layers(1,1) not correct";
  // EXPECT_TRUE(5==5);
}

TEST(FilterTests, medianfilter)
{
  cv::Mat img = cv::Mat::zeros(10, 10, EVENT_IMAGE_TYPE);
  img.at<uchar>(5,5) = 10;
  img.at<uchar>(4,5) = 5;
  img.at<uchar>(4,4) = 1;

  cv::Mat filtered = emvs::medianFilterNonZero(img, 3);
  int val_55 = filtered.at<uchar>(5,5);

  EXPECT_EQ(val_55, 5) << "filtered(5,5) not correct";
}

int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
