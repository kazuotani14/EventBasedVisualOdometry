// Testing PCL publishing, combining pointclouds, radius outlier removal

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
	ros::init (argc, argv, "test_pcl");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("points", 1);
	ros::Publisher pub_filtered = nh.advertise<sensor_msgs::PointCloud2> ("points_filtered", 1);

	// Make some random point clouds and concatenate them
  	PointCloud cloud1, cloud2, cloud3;
	cloud1.height = 30;
	cloud1.width = 1;
	cloud1.points.resize(cloud1.width * cloud1.height);

	cloud2.height = 30;
	cloud2.width = 1;
	cloud2.points.resize(cloud2.width * cloud2.height);

	for (size_t i = 0; i < cloud1.points.size (); ++i)
	{
	  cloud1.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
	  cloud1.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
	  cloud1.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	  cloud2.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
	  cloud2.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
	  cloud2.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}

	std::cout << "Cloud 1: " << std::endl;
	for (size_t i = 0; i < cloud1.points.size (); ++i)
	  std::cout << "    " << cloud1.points[i].x << " " << cloud1.points[i].y << " " << cloud1.points[i].z << std::endl;

	std::cout << "Cloud 2: " << std::endl;
  	for (size_t i = 0; i < cloud2.points.size (); ++i)
  	  std::cout << "    " << cloud2.points[i].x << " " << cloud2.points[i].y << " " << cloud2.points[i].z << std::endl;

	cloud3 = cloud1;
	cloud3 += cloud2;
	std::cout << "Cloud 3: " << std::endl;
  	for (size_t i = 0; i < cloud3.points.size (); ++i)
  	  std::cout << "    " << cloud3.points[i].x << " " << cloud3.points[i].y << " " << cloud3.points[i].z << std::endl;

	//Radius filter the resulting pointcloud
	PointCloud cloud_filtered;
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;

	// build and run the filter
	PointCloud::Ptr cloud_ptr(&cloud3);
	outrem.setInputCloud(cloud_ptr);
	outrem.setRadiusSearch(0.8);
	outrem.setMinNeighborsInRadius(2);
	outrem.filter(cloud_filtered);

    std::cout << "width: " << cloud3.width << " height: " << cloud3.height << std::endl;

	// Convert pointclouds to ros msgs and publish
	sensor_msgs::PointCloud2 msg, msg_filtered;
	pcl::toROSMsg(cloud3, msg);
	pcl::toROSMsg(cloud_filtered, msg_filtered);
	msg.header.frame_id = msg_filtered.header.frame_id = "world";
	// ROS_INFO("h: %d, w: %d", msg.height, msg.width);

	ROS_INFO("Publishing pointclouds, open rviz to see them");
	ros::Rate loop_rate(4);
	while (nh.ok())
	{
	  msg.header.stamp = msg_filtered.header.stamp = ros::Time::now();
	  pub.publish(msg);
	  pub_filtered.publish(msg_filtered);
	  ros::spinOnce();
	  loop_rate.sleep();
	}
}
