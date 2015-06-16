//#include <iostream>
//#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>
//#include <rospack/rospack.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>


using namespace std;
using namespace sensor_msgs;
typedef pcl::PointXYZRGB PointT;
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

int main (int argc, char** argv)
{
    ros::init(argc, argv, "pcd2pointcloud", ros::init_options::AnonymousName);
    ros::NodeHandle nh;


    cloud = (pcl::PointCloud<PointT>::Ptr ) new pcl::PointCloud<PointT>;

    string input; 
    if (ros::param::get("~input", input))
    {
        ROS_INFO("Loading point cloud from file %s",input.c_str());
        if (pcl::io::loadPCDFile<PointT>(input.c_str(), *cloud) != 0)
        {
            return -1;
        }
        cout << "Cloud has " << cloud->points.size () << " data points!" << endl;
    }
    else
    {
        ROS_ERROR("No point cloud filename given as input. use _input:=<filename>");
        exit(1);
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg (*cloud, msg);
    msg.header.frame_id = ros::names::remap("/map");

    string output="/camera/depth_registered/points"; 
    if (ros::param::get("~output", output))
    {
        ROS_INFO("Publishing point cloud on topic %s", output.c_str());
    }
    else
    {
        ROS_WARN("No topic name given, using default %s",output.c_str());
    }

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(output, 1);


    ros::Rate loop_rate(1);

    while (ros::ok())
    {

        pub.publish(msg);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return (0);
}

