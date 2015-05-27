#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


using namespace std;
using namespace octomap;
using namespace octomap_msgs;
using namespace sensor_msgs;

std::string output = "tmp.ot";
std::string input = "/octomap";

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{

    AbstractOcTree* tree = msgToMap(*msg);
    OcTree* octree = dynamic_cast<OcTree*>(tree);

    ROS_INFO("Saving octomap from topic %s to file %s", input.c_str(), output.c_str());
    octree->write(output.c_str());
    ros::Duration(1).sleep(); // sleep for half a second
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "octomap_msg2file", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    if (ros::param::get("~input", input))

    if (ros::param::get("~output", output))
    {
        ROS_INFO("Saving to file %s", output.c_str());
    }

    ROS_INFO("Subscribing to octree on topic name %s",input.c_str());
    ros::Subscriber sub = nh.subscribe(input, 1, octomapCallback);

    ros::spin();
    return (0);
}
