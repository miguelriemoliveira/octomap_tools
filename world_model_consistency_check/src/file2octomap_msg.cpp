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


using namespace std;
using namespace octomap;
using namespace octomap_msgs;
using namespace sensor_msgs;

OcTree* octree_model;

int main (int argc, char** argv)
{
    ros::init(argc, argv, "file2octomap_msg", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    std::string input; 
    if (ros::param::get("~input", input))
    {
        ROS_INFO("Loading octree from file %s",input.c_str());
        AbstractOcTree* tree = AbstractOcTree::read(input.c_str());
        if (!tree){
            return false;
        }
        octree_model = dynamic_cast<OcTree*>(tree);
    }
    else
    {
        ROS_ERROR("No octree filename given as input. use _input:=<filename>");
        exit(1);
    }

    std::string output="/output"; 
    if (ros::param::get("~output", output))
    {
        ROS_INFO("Publishing octree on topic %s", output.c_str());
    }
    else
    {
        ROS_WARN("No topic name given, using default %s",output.c_str());
    }

    int rate=1;
    ros::param::get("~rate", rate);

    octomap_msgs::Octomap msg;
    octomap_msgs::fullMapToMsg(*octree_model, msg);

    ros::Publisher pub = nh.advertise<octomap_msgs::Octomap>(output, 1);


    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        pub.publish(msg);
    }

    return (0);
}

