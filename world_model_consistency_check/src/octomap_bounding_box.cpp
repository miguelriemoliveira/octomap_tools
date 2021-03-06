#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// #include <octomap_msgs/GetOctomap.h>

#include <octomap_ros/conversions.h>
#include <octomap/OcTreeKey.h>

using namespace std;
using namespace octomap;
using namespace octomap_msgs;

AbstractOcTree *tree = NULL;
OcTree *octree = NULL;
OcTree *unknown_OcTree = NULL;
ros::Publisher marker_pub;
ros::Publisher cloud_pub;
ros::Publisher unknown_map;
ros::Publisher wireframe_pub;

point3d min_p, max_p;

std_msgs::ColorRGBA color_unknown, color_free, color_occupied;

string frame_id;

void wireframeCallback()
{
    visualization_msgs::MarkerArray marker_array;
    geometry_msgs::Point p;

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.ns = "wireframe";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0);

    marker.pose.position.x = (min_p.x() + max_p.x()) / 2;
    marker.pose.position.y = (min_p.y() + max_p.y()) / 2;
    marker.pose.position.z = (min_p.z() + max_p.z()) / 2;

    marker.scale.x = 0.01; //width of the line

    double x = fabs(min_p.x() - marker.pose.position.x);
    double y = fabs(min_p.y() - marker.pose.position.y);
    double z = fabs(min_p.z() - marker.pose.position.z);

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.7;
    marker.color.a = 1.0;

    p.x = x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);
    p.x = x;
    p.y = y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = x;
    p.y = y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);

    p.x = x;
    p.y = -y;
    p.z = z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = -y;
    p.z = z;
    marker.points.push_back(p);
    p.x = x;
    p.y = -y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = -y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = x;
    p.y = -y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = x;
    p.y = -y;
    p.z = z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = -y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = -y;
    p.z = z;
    marker.points.push_back(p);

    p.x = x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);
    p.x = x;
    p.y = -y;
    p.z = z;
    marker.points.push_back(p);
    p.x = x;
    p.y = y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = x;
    p.y = -y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = -y;
    p.z = z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = y;
    p.z = -z;
    marker.points.push_back(p);
    p.x = -x;
    p.y = -y;
    p.z = -z;
    marker.points.push_back(p);

    marker_array.markers.push_back(marker);

    wireframe_pub.publish(marker_array);
}

void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
    if (octree != NULL)
    {
        // // ROS_INFO("DEBUG: Going to DEL OCTREE");
        delete (octree);
        // // ROS_INFO("DEBUG: Octree DEL");
    }

    tree = msgToMap(*msg);
    octree = dynamic_cast<OcTree *>(tree);

    // std_msgs::ColorRGBA color_unknown, color_free, color_occupied;
    // string frame_id = "/base_link";
    ros::Time t = ros::Time::now();

    // point3d min;
    // min.x() = -0.5;
    // min.y() = -0.5;
    // min.z() = -0.5;

    // // point3d max;
    // max.x() = 0.5;
    // max.y() = 0.5;
    // max.z() = 0.5;

    // color_unknown.r = 1.0;
    // color_unknown.g = 0.6;
    // color_unknown.b = 0.0;
    // color_unknown.a = 0.3;

    // color_free.r = 0.0;
    // color_free.g = 1.0;
    // color_free.b = 0.0;
    // color_free.a = 0.3;

    // color_occupied.r = 1.0;
    // color_occupied.g = 0.0;
    // color_occupied.b = 0.0;
    // color_occupied.a = 1.0;

    // ros::param::get("~" + ros::names::remap("min_x"), min.x());
    // ros::param::get("~" + ros::names::remap("min_y"), min.y());
    // ros::param::get("~" + ros::names::remap("min_z"), min.z());

    // ros::param::get("~" + ros::names::remap("max_x"), max.x());
    // ros::param::get("~" + ros::names::remap("max_y"), max.y());
    // ros::param::get("~" + ros::names::remap("max_z"), max.z());

    // //ros::param::get("~resolution", resolution);
    // ros::param::get("~fixed_frame", frame_id);

    visualization_msgs::MarkerArray unknown_nodes, free_nodes, occupied_nodes;
    point3d_list unknown_centers, known_centers;
    pcl::PointCloud<pcl::PointXYZRGBA> unknown_pc;
    sensor_msgs::PointCloud2 unknown_cloud;

    octree->getUnknownLeafCenters(unknown_centers, min_p, max_p, 0);

    int received_tree_depth = octree->getTreeDepth();
    double received_tree_resolution = octree->getResolution();

    int num_of_unknown_cells = unknown_centers.size();
    unknown_nodes.markers.resize(received_tree_depth + 1);
    free_nodes.markers.resize(received_tree_depth + 1);
    occupied_nodes.markers.resize(received_tree_depth + 1);

    // ROS_INFO("Total unknown centers: %d", num_of_unknown_cells);

    KeySet unknown_cells;
    // octomap::OcTree *unknown_OcTree = new OcTree(received_tree_resolution);

    if (unknown_OcTree != NULL)
    {
        delete (unknown_OcTree);
    }

    unknown_OcTree = new OcTree(received_tree_resolution);

    // unknown_OcTree->setResolution(resolution);

    for (point3d_list::iterator it = unknown_centers.begin(); it != unknown_centers.end(); ++it)
    {

        pcl::PointXYZRGBA unknown_leaf_center_point;
        unknown_leaf_center_point.x = it->octomath::Vector3::x();
        unknown_leaf_center_point.y = it->octomath::Vector3::y();
        unknown_leaf_center_point.z = it->octomath::Vector3::z();
        unknown_leaf_center_point.r = 255 * color_unknown.r;
        unknown_leaf_center_point.g = 255 * color_unknown.g;
        unknown_leaf_center_point.b = 255 * color_unknown.b;
        unknown_leaf_center_point.a = 255 * color_unknown.a;
        unknown_pc.push_back(unknown_leaf_center_point);

        point3d insertionPoint(it->octomath::Vector3::x(), it->octomath::Vector3::y(), it->octomath::Vector3::z());
        OcTreeKey nodeKey = unknown_OcTree->coordToKey(insertionPoint);
        unknown_cells.insert(nodeKey);
    }

    // ROS_INFO("Number of unknown cells: %d", unknown_cells.size());

    pcl::toROSMsg(unknown_pc, unknown_cloud);
    unknown_cloud.header.frame_id = frame_id;
    unknown_cloud.header.stamp = t;

    for (KeySet::iterator it = unknown_cells.begin(), end = unknown_cells.end(); it != end; ++it)
    {
        unknown_OcTree->updateNode(*it, false); //mark as free voxel
    }

    unknown_OcTree->prune();

    // ROS_INFO("New OcTree number of leafs: %d", unknown_OcTree->getNumLeafNodes());

    double unknown_octree_depth = unknown_OcTree->getTreeDepth();

    for (OcTree::iterator it = unknown_OcTree->begin(unknown_octree_depth), end = unknown_OcTree->end(); it != end; ++it)
    {

        if (!unknown_OcTree->isNodeOccupied(*it))
        {
            double size = it.getSize();
            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();

            unsigned idx = it.getDepth();
            assert(idx < unknown_nodes.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            unknown_nodes.markers[idx].points.push_back(cubeCenter);
        }
    }

    // ROS_INFO("Total marker points pushed: %d", unknown_nodes.markers.size());

    for (unsigned i = 0; i < unknown_nodes.markers.size(); i++)
    {
        // cout << i << endl;
        double size = unknown_OcTree->getNodeSize(i);
        // double size = 0.05;

        unknown_nodes.markers[i].header.frame_id = frame_id;
        unknown_nodes.markers[i].header.stamp = t;
        unknown_nodes.markers[i].ns = "unknown_boxes";
        unknown_nodes.markers[i].id = i;
        unknown_nodes.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        unknown_nodes.markers[i].scale.x = size;
        unknown_nodes.markers[i].scale.y = size;
        unknown_nodes.markers[i].scale.z = size;
        unknown_nodes.markers[i].color = color_unknown;

        if (unknown_nodes.markers[i].points.size() > 0)
        {
            unknown_nodes.markers[i].action = visualization_msgs::Marker::ADD;
        }
        else
        {
            unknown_nodes.markers[i].action = visualization_msgs::Marker::DELETE;
        }
    }

    for (OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min_p, max_p), end = octree->end_leafs_bbx(); it != end; it++)
    {
        if (!octree->isNodeOccupied(*it))
        {
            double size = it.getSize();
            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();

            unsigned idx = it.getDepth();
            assert(idx < free_nodes.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            free_nodes.markers[idx].points.push_back(cubeCenter);
        }
        else
        {
            double size = it.getSize();
            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();

            unsigned idx = it.getDepth();
            assert(idx < occupied_nodes.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            occupied_nodes.markers[idx].points.push_back(cubeCenter);
        }
    }

    for (unsigned i = 0; i < free_nodes.markers.size(); i++)
    {
        // cout << i << endl;
        double size = octree->getNodeSize(i);
        // double size = 0.05;

        free_nodes.markers[i].header.frame_id = frame_id;
        free_nodes.markers[i].header.stamp = t;
        free_nodes.markers[i].ns = "free_boxes";
        free_nodes.markers[i].id = i;
        free_nodes.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        free_nodes.markers[i].scale.x = size;
        free_nodes.markers[i].scale.y = size;
        free_nodes.markers[i].scale.z = size;
        free_nodes.markers[i].color = color_free;

        if (free_nodes.markers[i].points.size() > 0)
        {
            free_nodes.markers[i].action = visualization_msgs::Marker::ADD;
        }
        else
        {
            free_nodes.markers[i].action = visualization_msgs::Marker::DELETE;
        }
    }

    for (unsigned i = 0; i < occupied_nodes.markers.size(); i++)
    {
        // cout << i << endl;
        double size = octree->getNodeSize(i);
        // double size = 0.05;

        occupied_nodes.markers[i].header.frame_id = frame_id;
        occupied_nodes.markers[i].header.stamp = t;
        occupied_nodes.markers[i].ns = "occupied_boxes";
        occupied_nodes.markers[i].id = i;
        occupied_nodes.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        occupied_nodes.markers[i].scale.x = size;
        occupied_nodes.markers[i].scale.y = size;
        occupied_nodes.markers[i].scale.z = size;
        occupied_nodes.markers[i].color = color_occupied;

        if (occupied_nodes.markers[i].points.size() > 0)
        {
            occupied_nodes.markers[i].action = visualization_msgs::Marker::ADD;
        }
        else
        {
            occupied_nodes.markers[i].action = visualization_msgs::Marker::DELETE;
        }
    }

    wireframeCallback();

    cloud_pub.publish(unknown_cloud);

    marker_pub.publish(unknown_nodes);
    marker_pub.publish(free_nodes);
    marker_pub.publish(occupied_nodes);

    Octomap map;
    map.header.frame_id = frame_id;
    map.header.stamp = t;

    if (octomap_msgs::fullMapToMsg(*unknown_OcTree, map))
    {
        unknown_map.publish(map);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_bounding_box_node");
    ros::NodeHandle nh;

    min_p.x() = -0.5;
    min_p.y() = -0.5;
    min_p.z() = -0.5;

    max_p.x() = 0.5;
    max_p.y() = 0.5;
    max_p.z() = 0.5;

    color_unknown.r = 1.0;
    color_unknown.g = 0.6;
    color_unknown.b = 0.0;
    color_unknown.a = 0.3;

    color_free.r = 0.0;
    color_free.g = 1.0;
    color_free.b = 0.0;
    color_free.a = 0.3;

    color_occupied.r = 1.0;
    color_occupied.g = 0.0;
    color_occupied.b = 0.0;
    color_occupied.a = 1.0;

    frame_id = "/base_link";

    double rate = 1;
    ros::param::get("~rate", rate);

    ros::param::get("~" + ros::names::remap("min_x"), min_p.x());
    ros::param::get("~" + ros::names::remap("min_y"), min_p.y());
    ros::param::get("~" + ros::names::remap("min_z"), min_p.z());

    ros::param::get("~" + ros::names::remap("max_x"), max_p.x());
    ros::param::get("~" + ros::names::remap("max_y"), max_p.y());
    ros::param::get("~" + ros::names::remap("max_z"), max_p.z());

    //ros::param::get("~resolution", resolution);
    ros::param::get("~fixed_frame", frame_id);

    ros::Duration(0.5).sleep();

    ros::Subscriber sub = nh.subscribe("/octomap_full", 1, octomapCallback);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/cells_vis_array", 10);

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/unknown_pc", 10);
    unknown_map = nh.advertise<octomap_msgs::Octomap>("/unknown_full_map", 10);

    wireframe_pub = nh.advertise<visualization_msgs::MarkerArray>("/octomap_boundingBox", 100);

    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
