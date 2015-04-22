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

OcTree* octree_model = NULL;
OcTree* octree_target = NULL;

std::string topic_model = "/octomap_full";
std::string topic_target = "/output";


void octomapCallbackModel(const octomap_msgs::Octomap::ConstPtr& msg)
{

    AbstractOcTree* tree = msgToMap(*msg);
    octree_model = dynamic_cast<OcTree*>(tree);


    ROS_INFO("\n\nReceived new octree model on topic %s with id: %s, frame_id: %s\n\n.", topic_model.c_str(), msg->id.c_str(), msg->header.frame_id.c_str());

    // // int treeDepth = octree_model->getTreeDepth();
    // // ROS_INFO("treeDepth = %d", treeDepth);
    
}


void octomapCallbackTarget(const octomap_msgs::Octomap::ConstPtr& msg)
{

    AbstractOcTree* tree = msgToMap(*msg);
    octree_target = dynamic_cast<OcTree*>(tree);

    ROS_INFO("\n\nReceived new octree target on topic %s with id: %s, frame_id: %s\n\n.", topic_target.c_str(), msg->id.c_str(), msg->header.frame_id.c_str());

    // int treeDepth = octree_target->getTreeDepth();
    // ROS_INFO("treeDepth = %d", treeDepth);

    if (octree_model == NULL)
    {
        ROS_INFO("OcTree Model Not Found");

        return;
    }

    // Bounding Box and Resolution Initialization
    point3d min; min.x() = 0; min.y() = 0; min.z() = 0;
    point3d max; max.x() = 0; max.y() = 0; max.z() = 0;
    

    // Visualization Message Marker Array
    visualization_msgs::MarkerArray ma;
    int id=0;

    ROS_INFO_STREAM("Starting Iteration");

    // OcTree Target Iterator
    for(OcTree::tree_iterator it = octree_target->begin_tree(), end=octree_target->end_tree(); it!= end; ++it)
    {
        // Verifies if the Node exists
        if (octree_target->search(it.getKey()))
        {
            // ROS_INFO("Found an known node!");
            
            // Verifies if the Node is occupied
            //if (it->getValue() > 0)
            if (octree_target->isNodeOccupied(*it))
            {
                // ROS_INFO("Found an known and occupied node!");
                // cout << "Found a know Node with value: " << it->getValue() << endl;

                // Defines Bounding Box
                double resolution = it.getSize();
                min.x() = it.getX()-resolution; min.y() = it.getY()-resolution; min.z() = it.getZ()-resolution;
                max.x() = it.getX()+resolution; max.y() = it.getY()+resolution; max.z() = it.getZ()+resolution;

                // OcTree Model Iterator with Bounding Box
                for(OcTree::leaf_bbx_iterator it_model = octree_model->begin_leafs_bbx(min,max), end=octree_model->end_leafs_bbx(); it_model!= end; ++it_model)
                {
                    // Verifies if the Nodes exists
                    if (octree_model->search(it_model.getKey()))
                    {
                        // Verifies if the Node is not occupied
                        //if (it_model->getValue() < 0)
                        if (octree_model->isNodeOccupied(*it_model))
                        {
                            // Draws the Node in the marker array
                            visualization_msgs::Marker m;

                            //ROS_INFO_STREAM("found!");

                            m.ns = "boxes";
                            m.header.frame_id = "kinect_rgb_optical_frame";
                            m.header.stamp = ros::Time::now();
                            m.action = visualization_msgs::Marker::ADD;
                            //m.pose.orientation.w = 1.0;
                            m.id = id++;
                            //m.lifetime = 0;
                            m.type = visualization_msgs::Marker::CUBE;
                            m.scale.x = it.getSize();
                            m.scale.y = it.getSize();
                            m.scale.z = it.getSize();

                            m.pose.position.x = it.getX();
                            m.pose.position.y = it.getY();
                            m.pose.position.z = it.getZ();
                            m.color.r = 0.0;
                            m.color.g = 1.0;
                            m.color.b = 1.0;
                            m.color.a = 0.9;

                            ma.markers.push_back(m);

                        }
                    }
                }
            }
        }
    }

    marker_pub->publish(ma);

}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "compare_octrees");
    ros::NodeHandle nh;

    // Use: _topic_model:=/topic_model  and  _topic_target:=/topic_target
    ros::param::get("~topic_model", topic_model);
    ros::param::get("~topic_target", topic_target);

    //Problem linking? check http://answers.ros.org/question/196935/roslib-reference-error/
    //string path = ros::package::getPath("amazon_object_segmentation");

    ros::Duration(1).sleep(); // sleep for a second

    ros::Subscriber sub_model = nh.subscribe(topic_model, 1, octomapCallbackModel);
    ros::Subscriber sub_target = nh.subscribe(topic_target, 1, octomapCallbackTarget);

    marker_pub = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/inconsistencies_arrays", 10);

    ros::spin();
    return (0);
}
