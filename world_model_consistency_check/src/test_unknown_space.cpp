//#include <iostream>
//#include <boost/thread/thread.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <rospack/rospack.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
//#include <octomap/OcTree.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>

typedef pcl::PointCloud<pcl::PointXYZ> PC;

using namespace std;
using namespace octomap;
using namespace octomap_msgs;
using namespace sensor_msgs;

boost::shared_ptr<ros::Publisher> pub;
boost::shared_ptr<ros::Publisher> marker_pub;

// Memory Leak Fix
AbstractOcTree* tree = NULL;
OcTree* octree = NULL;

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
  ROS_INFO("\n\nReceived new octomap with id: %s, frame_id: %s\n\n", msg->id.c_str(), msg->header.frame_id.c_str());

  // AbstractOcTree* tree = msgToMap(*msg);
  // OcTree* octree = dynamic_cast<OcTree*>(tree);
  string frame_id = "/base_link";

  // Memory Leak Fix
//   if (tree != NULL)
//   {
//     ROS_INFO("DEBUG: Going to DEL TREE");
//     delete (tree);
//     ROS_INFO("DEBUG: Tree DEL");
//   }

  if (octree != NULL)
  {
    // ROS_INFO("DEBUG: Going to DEL OCTREE");
    delete (octree);
    // ROS_INFO("DEBUG: Octree DEL");
  }

  tree = msgToMap(*msg);
  octree = dynamic_cast<OcTree*>(tree);
  //

  // point3d min; min.x() = 0; min.y() = 0; min.z() = 0;
  point3d min;
  min.x() = 0.59;
  min.y() = -1.26;
  min.z() = -0.73;
  cout << min << endl;
  point3d max;
  max.x() = 1.92;
  max.y() = 1.20;
  max.z() = 0.2;

  ros::param::get("~min_x", min.x());
  ros::param::get("~min_y", min.y());
  ros::param::get("~min_z", min.z());

  ros::param::get("~max_x", max.x());
  ros::param::get("~max_y", max.y());
  ros::param::get("~max_z", max.z());

  cout << max << endl;
  double resolution = 0.15;

  ros::param::get("~resolution", resolution);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_msg(new pcl::PointCloud<pcl::PointXYZ>);

  ros::param::get("~fixed_frame", frame_id);

  pc_msg->header.frame_id = frame_id;
  pc_msg->height = 1;
  pc_msg->width = 0;

  visualization_msgs::MarkerArray ma;
  int id = 0;

  int treeDepth = octree->getTreeDepth();
  ROS_INFO("treeDepth = %d", treeDepth);

  ros::Time t = ros::Time::now();

  id = 0;

  for (double ix = min.x(); ix < max.x(); ix += resolution)
    for (double iy = min.y(); iy < max.y(); iy += resolution)
      for (double iz = min.z(); iz < max.z(); iz += resolution)
      {
        if (!octree->search(ix, iy, iz))
        {
          visualization_msgs::Marker m;

          m.ns = "boxes";
          m.header.frame_id = frame_id;
          m.header.stamp = t;
          m.action = visualization_msgs::Marker::ADD;
          // m.pose.orientation.w = 1.0;
          m.id = id++;
          // m.lifetime = 0;
          m.type = visualization_msgs::Marker::CUBE;
          m.scale.x = m.scale.y = m.scale.z = resolution;

          m.pose.position.x = ix;
          m.pose.position.y = iy;
          m.pose.position.z = iz;
          m.color.r = 1.0;
          m.color.g = 0.6;
          m.color.b = 0.0;
          m.color.a = 0.3;

          ma.markers.push_back(m);
        }
      }
  /*
    visualization_msgs::Marker m;
    m.ns = "target_volume";
    m.header.frame_id = frame_id;
    m.header.stamp = t;
    m.action = visualization_msgs::Marker::ADD;
    // m.pose.orientation.w = 1.0;
    m.id = id++;
    // m.lifetime = 0;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.scale.x = .005;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    geometry_msgs::Point p1;

    p1.x = min.x();
    p1.y = min.y();
    p1.z = min.z();
    m.points.push_back(p1);
    p1.x = max.x();
    p1.y = min.y();
    p1.z = min.z();
    m.points.push_back(p1);
    p1.x = max.x();
    p1.y = max.y();
    p1.z = min.z();
    m.points.push_back(p1);
    p1.x = min.x();
    p1.y = max.y();
    p1.z = min.z();
    m.points.push_back(p1);
    p1.x = min.x();
    p1.y = min.y();
    p1.z = min.z();
    m.points.push_back(p1);
    p1.x = min.x();
    p1.y = min.y();
    p1.z = max.z();
    m.points.push_back(p1);
    p1.x = max.x();
    p1.y = min.y();
    p1.z = max.z();
    m.points.push_back(p1);
    p1.x = max.x();
    p1.y = max.y();
    p1.z = max.z();
    m.points.push_back(p1);
    p1.x = min.x();
    p1.y = max.y();
    p1.z = max.z();
    m.points.push_back(p1);
    p1.x = min.x();
    p1.y = min.y();
    p1.z = max.z();
    m.points.push_back(p1);
    p1.x = min.x();
    p1.y = max.y();
    p1.z = max.z();
    m.points.push_back(p1);
    p1.x = min.x();
    p1.y = max.y();
    p1.z = min.z();
    m.points.push_back(p1);
    p1.x = max.x();
    p1.y = max.y();
    p1.z = min.z();
    m.points.push_back(p1);
    p1.x = max.x();
    p1.y = max.y();
    p1.z = max.z();
    m.points.push_back(p1);
    p1.x = max.x();
    p1.y = min.y();
    p1.z = max.z();
    m.points.push_back(p1);
    p1.x = max.x();
    p1.y = min.y();
    p1.z = min.z();
    m.points.push_back(p1);

    ma.markers.push_back(m);

    // OcTreeKey minKey(0,0,0);
    // OcTreeKey maxKey(0,0,0);
    // octree->coordToKeyChecked(min, minKey);
    // octree->coordToKeyChecked(max, maxKey);
    // OcTreeKey k;
    // for (k[0] = minKey[0]; k[0] < maxKey[0]; ++k[0]){
    // for (k[1] = minKey[1]; k[1] < maxKey[1]; ++k[1]){
    // for (k[2] = minKey[2]; k[2] < maxKey[2]; ++k[2]){
    // OcTreeNode* n = octree->search(k);
    // if(!n)
    //{
    ////visualization_msgs::Marker m;

    ////m.ns = "boxes";
    ////m.header.frame_id = "/base_link";
    ////m.header.stamp = ros::Time::now();
    ////m.action = visualization_msgs::Marker::ADD;
    //////m.pose.orientation.w = 1.0;
    ////m.id = id++;
    //////m.lifetime = 0;
    ////m.type = visualization_msgs::Marker::CUBE;
    ////m.scale.x = k->getSize();
    ////m.scale.y = it.getSize();
    ////m.scale.z = it.getSize();

    ////m.pose.position.x = it.getX();
    ////m.pose.position.y = it.getY();
    ////m.pose.position.z = it.getZ();
    ////m.color.r = 0.0;
    ////m.color.g = 1.0;
    ////m.color.b = 1.0;
    ////m.color.a = 0.9;

    ////ma.markers.push_back(m);
    //}
    //}
    //}
    //}

    // octree->write("text.ot");
    // for (OcTree::iterator it = octree->begin(treeDepth), end = octree->end(); it != end; ++it)
    // for(OcTree::tree_iterator it = octree->begin_tree(), end=octree->end_tree(); it!= end; ++it)
    ////for(OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min,max), end=octree->end_leafs_bbx(); it!= end;
    ++it)
    ////for(OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it!= end; ++it)
    //{

    ////Check that cell is inside the bounding box
    ////if (it.getX() < min.x() || it.getX() > max.x()) continue;
    ////if (it.getY() < min.y() || it.getY() > max.y()) continue;
    ////if (it.getZ() < min.z() || it.getZ() > max.z()) continue;

    ////if (octree->search(it.getX(), it.getY(), it.getZ()) == NULL)
    // if (!octree->search(it.getKey()))
    //{
    // ROS_ERROR("Found an unknown node");
    // //cout << "Node center: " << it.getCoordinate() << endl;
    // //cout << "Node size: " << it.getSize() << endl;
    // //cout << "Node value: " << it->getValue() << endl;

    // pc_msg->points.push_back (pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
    // pc_msg->width++;

    // visualization_msgs::Marker m;

    // m.ns = "boxes";
    // m.header.frame_id = "/base_link";
    // m.header.stamp = ros::Time::now();
    // m.action = visualization_msgs::Marker::ADD;
    ////m.pose.orientation.w = 1.0;
    // m.id = id++;
    ////m.lifetime = 0;
    // m.type = visualization_msgs::Marker::CUBE;
    // m.scale.x = it.getSize();
    // m.scale.y = it.getSize();
    // m.scale.z = it.getSize();

    // m.pose.position.x = it.getX();
    // m.pose.position.y = it.getY();
    // m.pose.position.z = it.getZ();
    // m.color.r = 0.0;
    // m.color.g = 1.0;
    // m.color.b = 1.0;
    // m.color.a = 0.9;

    // ma.markers.push_back(m);
    //}
    //}

    // pc_msg->header.stamp = ros::Time::now();
    // pub->publish(pc_msg);
  */
  marker_pub->publish(ma);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrate_reference_system_in_model_node");
  ros::NodeHandle nh;

  // Problem linking? check http://answers.ros.org/question/196935/roslib-reference-error/
  // string path = ros::package::getPath("amazon_object_segmentation");

  ros::Duration(1).sleep();  // sleep for half a second

  ros::Subscriber sub = nh.subscribe("/octomap_full", 1, octomapCallback);

  pub = (boost::shared_ptr<ros::Publisher>)(new ros::Publisher);
  *pub = nh.advertise<PointCloud2>("points2", 1);

  marker_pub = (boost::shared_ptr<ros::Publisher>)(new ros::Publisher);
  *marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/unknown_space", 10);

  ros::spin();
  return (0);
}
