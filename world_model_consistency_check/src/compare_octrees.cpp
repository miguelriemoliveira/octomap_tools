//#include <iostream>
//#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>
//#include <rospack/rospack.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>
//#include <octomap/OcTree.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;
using namespace octomap;
using namespace octomap_msgs;
using namespace sensor_msgs;

boost::shared_ptr<ros::Publisher> pub;
boost::shared_ptr<ros::Publisher> marker_pub;

OcTree* octree_model;
OcTree* octree_target;


void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{

    AbstractOcTree* tree = msgToMap(*msg);
    OcTree* octree_model = dynamic_cast<OcTree*>(tree);

    ROS_INFO("\n\nReceived new octree model with id: %s, frame_id: %s\n\n.", msg->id.c_str(), msg->header.frame_id.c_str());

}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "calibrate_reference_system_in_model_node");
    ros::NodeHandle nh;

    //Problem linking? check http://answers.ros.org/question/196935/roslib-reference-error/
    //string path = ros::package::getPath("amazon_object_segmentation");

    ros::Duration(1).sleep(); // sleep for half a second

    ros::Subscriber sub = nh.subscribe("/octomap_full", 1, octomapCallback);

    marker_pub = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/marker_array", 10);

    ros::spin();
    return (0);
}
