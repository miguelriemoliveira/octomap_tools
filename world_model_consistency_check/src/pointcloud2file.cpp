#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZRGBA PointT;

std::string filename = "output.pcd";

void
cloud_open_target (const sensor_msgs::PointCloud2ConstPtr& msg)
{

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  pcl::fromROSMsg (*msg, *cloud);

  ROS_INFO("Saving to file %s", filename.c_str());;

  pcl::io::savePCDFileASCII (filename, *cloud);

}


int
main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "ros_change_detection");

  // Node Handle
  ros::NodeHandle nh;

  ros::param::get("~filename", filename);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_target = nh.subscribe ("input", 1, cloud_open_target);

  // Spin
  ros::spin ();

}