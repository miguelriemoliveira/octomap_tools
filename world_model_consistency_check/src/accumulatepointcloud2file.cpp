#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZRGBA PointT;

std::string filename = "output.pcd";

pcl::PointCloud<PointT>::Ptr accumulated_cloud;
//pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

void cloud_open_target (const sensor_msgs::PointCloud2ConstPtr& msg)
{
   // sensor_msgs::PointCloud2 msg_filtered;
   // sor.setInputCloud (*msg);
   // sor.filter (*msg_filtered);

    
    

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg (*msg, *cloud);

    pcl::VoxelGrid<PointT> grid;
    //pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize (0.01f, 0.01f, 0.01f);
    grid.setInputCloud (cloud);
    grid.filter (*cloud_filtered);



    (*accumulated_cloud) += (*cloud);
    //ROS_INFO("Size of accumulated_cloud = %ld", accumulated_cloud->points.size());
}


int main (int argc, char** argv)
{

    // Initialize ROS
    ros::init (argc, argv, "ros_change_detection");

    // Node Handle
    ros::NodeHandle nh;

    ros::param::get("~filename", filename);


    accumulated_cloud = (pcl::PointCloud<PointT>::Ptr) new pcl::PointCloud<PointT>;

    //sor.setLeafSize (0.01f, 0.01f, 0.01f);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub_target = nh.subscribe ("input", 1, cloud_open_target);


    while (ros::ok())
    {
        // Spin
        ros::spinOnce();
    }

    printf("Saving to file %s\n", filename.c_str());;
    pcl::io::savePCDFileASCII (filename, *accumulated_cloud);

}
