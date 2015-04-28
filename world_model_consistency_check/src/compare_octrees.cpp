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

#include <dynamic_reconfigure/server.h>
#include <world_model_consistency_check/DepthConfigurationConfig.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;
using namespace octomap;
using namespace octomap_msgs;
using namespace sensor_msgs;

/**
 * @brief Defines a cubic volume that we can use to limit an octree search
 */
class ClassBoundingBox
{
    public:

        ClassBoundingBox(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
        {
            _min_x = min_x; _max_x = max_x;
            _min_y = min_y; _max_y = max_y;
            _min_z = min_z; _max_z = max_z;
        };

        ClassBoundingBox(double center_x, double center_y, double center_z, double size)
        {
            double s2 = size/2;
            _min_x = center_x - s2; _max_x = center_x + s2;
            _min_y = center_y - s2; _max_y = center_y + s2;
            _min_z = center_z - s2; _max_z = center_z + s2;
        };


        ~ClassBoundingBox(){};

        point3d getMinimumPoint(void){point3d p; p.x() = _min_x; p.y() = _min_y; p.z() = _min_z; return p;};
        point3d getMaximumPoint(void){point3d p; p.x() = _max_x; p.y() = _max_y; p.z() = _max_z; return p;};

        void getEdgesToDraw(std::vector<geometry_msgs::Point>& l)
        {
            geometry_msgs::Point p;
            p.x = _min_x; p.y=_min_y; p.z = _min_z; l.push_back(p);
            p.x = _max_x; p.y=_min_y; p.z = _min_z; l.push_back(p);
            p.x = _max_x; p.y=_max_y; p.z = _min_z; l.push_back(p);
            p.x = _min_x; p.y=_max_y; p.z = _min_z; l.push_back(p);
            p.x = _min_x; p.y=_min_y; p.z = _min_z; l.push_back(p);
            p.x = _min_x; p.y=_min_y; p.z = _max_z; l.push_back(p);
            p.x = _max_x; p.y=_min_y; p.z = _max_z; l.push_back(p);
            p.x = _max_x; p.y=_max_y; p.z = _max_z; l.push_back(p);
            p.x = _min_x; p.y=_max_y; p.z = _max_z; l.push_back(p);
            p.x = _min_x; p.y=_min_y; p.z = _max_z; l.push_back(p);
            p.x = _min_x; p.y=_max_y; p.z = _max_z; l.push_back(p);
            p.x = _min_x; p.y=_max_y; p.z = _min_z; l.push_back(p);
            p.x = _max_x; p.y=_max_y; p.z = _min_z; l.push_back(p);
            p.x = _max_x; p.y=_max_y; p.z = _max_z; l.push_back(p);
            p.x = _max_x; p.y=_min_y; p.z = _max_z; l.push_back(p);
            p.x = _max_x; p.y=_min_y; p.z = _min_z; l.push_back(p);
        }

        visualization_msgs::Marker createMarker(std::string ns, std::string frame_id, std_msgs::ColorRGBA color, int id)
        {
            visualization_msgs::Marker m;
            m.ns = ns;
            m.header.frame_id = frame_id;
            m.header.stamp = ros::Time::now();
            m.action = visualization_msgs::Marker::ADD;
            m.id = id;
            m.color = color;
            return m;
        }


        visualization_msgs::Marker getMarkerWithEdges(std::string ns, std::string frame_id, std_msgs::ColorRGBA color, int id)
        {
            visualization_msgs::Marker m = createMarker(ns, frame_id, color, id);
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.scale.x = 0.002;
            getEdgesToDraw(m.points);
            return m;
        }

        visualization_msgs::Marker getMarkerCubeVolume(std::string ns, std::string frame_id, std_msgs::ColorRGBA color, int id)
        {

            visualization_msgs::Marker m = createMarker(ns, frame_id, color, id);
            m.type = visualization_msgs::Marker::CUBE;
            double size = _max_x - _min_x;
            m.scale.x = m.scale.y = m.scale.z = size;
            m.pose.position.x = (_max_x + _min_x)/2;
            m.pose.position.y = (_max_y + _min_y)/2;
            m.pose.position.z = (_max_z + _min_z)/2;
            return m;
        }


    protected:

        double _min_x;
        double _max_x;
        double _min_y;
        double _max_y;
        double _min_z;
        double _max_z;
};


/* _________________________________
  |                                 |
  |        GLOBAL VARIABLES         |
  |_________________________________| */


boost::shared_ptr<ros::Publisher> pub;
boost::shared_ptr<ros::Publisher> marker_pub;

OcTree* octree_model = NULL;
OcTree* octree_target = NULL;

std::string topic_model = "/octomap_full";
std::string topic_target = "/output";

unsigned char depth = 16;

//Declare a ClassBoundingBox which defines the target_volume
ClassBoundingBox target_volume(-0.42, 0.42, -0.42, 0.42, 0.2, 3.0);

void octomapCallbackModel(const octomap_msgs::Octomap::ConstPtr& msg)
{
    AbstractOcTree* tree = msgToMap(*msg);
    octree_model = dynamic_cast<OcTree*>(tree);

    ROS_INFO("Received new octree model on topic %s with id: %s, frame_id: %s\n", topic_model.c_str(), msg->id.c_str(), msg->header.frame_id.c_str());

    // // int treeDepth = octree_model->getTreeDepth();
    // // ROS_INFO("treeDepth = %d", treeDepth);
}

//visualization_msgs::Marker createCubeMarker(point3d center, double size, std::string ns, std::string frame_id, std_msgs::ColorRGBA color)
//{
    //static int id=7777;

    //visualization_msgs::Marker m;
    //m.ns = ns;
    //m.header.frame_id = frame_id;
    //m.header.stamp = ros::Time::now();
    //m.action = visualization_msgs::Marker::ADD;
    ////m.pose.orientation.w = 1.0;
    //m.id = id++;
    ////m.lifetime = 0;
    //m.type = visualization_msgs::Marker::CUBE;
    //m.scale.x = size;
    //m.scale.y = size;
    //m.scale.z = size;

    //m.pose.position.x = center.x();
    //m.pose.position.y = center.y();
    //m.pose.position.z = center.z();
    //m.color = color;
    //return m;
//}


void octomapCallbackTarget(const octomap_msgs::Octomap::ConstPtr& msg)
{
    AbstractOcTree* tree = msgToMap(*msg);
    octree_target = dynamic_cast<OcTree*>(tree);

    ROS_INFO("Received new octree target on topic %s with id: %s, frame_id: %s\n", topic_target.c_str(), msg->id.c_str(), msg->header.frame_id.c_str());

    // int treeDepth = octree_target->getTreeDepth();
    // ROS_INFO("treeDepth = %d", treeDepth);

}


void compareCallback(const ros::TimerEvent&)
{
    ros::Time t= ros::Time::now();

    ROS_INFO("Compare callback triggered");



    // Checks if the OcTree Model was already received
    if (octree_model == NULL)
    {
        ROS_INFO("OcTree Model Not Found");
        return;
    }

    // Checks if the OcTree Target was already received
    if (octree_target == NULL)
    {
        ROS_INFO("OcTree Target Not Found");
        return;   
    }

    // Visualization Message Marker Array
    visualization_msgs::MarkerArray ma;
    unsigned int id=0;
    std_msgs::ColorRGBA color_occupied;
    color_occupied.r = 0; color_occupied.g = 0; color_occupied.b = 0.5; color_occupied.a = .8;
    std_msgs::ColorRGBA color_inconsistent;
    color_inconsistent.r = .5; color_inconsistent.g = 0; color_inconsistent.b = 0; color_inconsistent.a = .8;
    std_msgs::ColorRGBA color_target_volume;
    color_target_volume.r = .5; color_target_volume.g = 0.5; color_target_volume.b = 0; color_target_volume.a = 1;

    std::vector<ClassBoundingBox> v_inconsistencies;


    ma.markers.push_back(target_volume.getMarkerWithEdges("target_volume", "kinect_rgb_optical_frame", color_target_volume, ++id));



    ROS_INFO_STREAM("Starting Iteration");

    // -------------------------------------------------------------
    // ----------- Iterate over target octree ----------------------
    // -------------------------------------------------------------
    for(OcTree::leaf_bbx_iterator it = octree_target->begin_leafs_bbx(target_volume.getMinimumPoint(), target_volume.getMaximumPoint(), depth), end=octree_target->end_leafs_bbx(); it!= end; ++it)
    {
        if (octree_target->search(it.getKey())) // Verifies if the node exists
        {
            if (octree_target->isNodeOccupied(*it)) // Verifies if the Node is occupied
            {
                //ROS_INFO("Found an known and occupied node!");
                ClassBoundingBox target_cell(it.getX(), it.getY(), it.getZ(), it.getSize());
                ma.markers.push_back(target_cell.getMarkerWithEdges("target_occupied", "kinect_rgb_optical_frame", color_occupied, ++id));

                bool flg_found_occupied = false;

                // -------------------------------------------------------------
                // ----------- Iterate over model octree ----------------------
                // -------------------------------------------------------------
                for(OcTree::leaf_bbx_iterator it_model = octree_model->begin_leafs_bbx(target_cell.getMinimumPoint(),target_cell.getMaximumPoint(), depth), end=octree_model->end_leafs_bbx(); it_model!= end; ++it_model)
                {
                    if (octree_model->search(it_model.getKey())) // Verifies if the nodes exists
                    {
                        if (!octree_model->isNodeOccupied(*it_model)) // Verifies if the node is free
                        {
                            //Do something here - Draw node, etc
                            //ClassBoundingBox inconsistent_volume(it_model.getX(), it_model.getY(), it_model.getZ(), it_model.getSize());
                            //inconsistent_volume.getEdgesToDraw(m.points);
                            //ma.markers.push_back(m);

                            //ROS_INFO_STREAM("found!");
                        }
                        else
                        {
                            flg_found_occupied = true;
                        }
                    }
                }

                if (flg_found_occupied == false) //If no occupied cell was found out of all iterated in the model's bbox, then an inconsistency is detected
                {
                    v_inconsistencies.push_back(target_cell);
                    ma.markers.push_back(target_cell.getMarkerCubeVolume("target_inconsistent", "kinect_rgb_optical_frame", color_inconsistent, ++id));
                }
            }
        }
    }

    marker_pub->publish(ma);


    //Do your stuff

    ROS_INFO("Inconsistencies vecotr har %ld cells", v_inconsistencies.size());

    ros::Duration d = (ros::Time::now() - t);
    ROS_INFO("Comparisson took %f secs", d.toSec());
}


void callbackDynamicReconfigure(world_model_consistency_check::DepthConfigurationConfig &config, uint32_t level) 
{
    ROS_INFO("Reconfigure Request: Setting comparison depth to %d",  config.depth);
    depth = (unsigned char) config.depth;
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "compare_octrees");
    ros::NodeHandle nh;

    //The c way
    //int v[20];
    //int* v;
    //v = malloc(sizeof(int) *200);
    //v[0] = 20;
    //...

    //v[201] = 21; --> segmentation fault


    //free(v);

    ////The c++ way
    //std::vector<int> v;
    //v.push_back(20)
        //...
    //v.push_back(21)

    //std::vector<ClassBoundingBox> v;

    //return 1;

    // Use: _topic_model:=/topic_model  and  _topic_target:=/topic_target
    ros::param::get("~topic_model", topic_model);
    ros::param::get("~topic_target", topic_target);

    //Problem linking? check http://answers.ros.org/question/196935/roslib-reference-error/
    //string path = ros::package::getPath("amazon_object_segmentation");

    //Setup the depth online configuration
    dynamic_reconfigure::Server<world_model_consistency_check::DepthConfigurationConfig> server;
    dynamic_reconfigure::Server<world_model_consistency_check::DepthConfigurationConfig>::CallbackType f;
    f = boost::bind(&callbackDynamicReconfigure, _1, _2);
    server.setCallback(f);


    ros::Duration(1).sleep(); // sleep for a second

    ros::Subscriber sub_model = nh.subscribe(topic_model, 2, octomapCallbackModel);
    ros::Subscriber sub_target = nh.subscribe(topic_target, 2, octomapCallbackTarget);

    ros::Timer timer = nh.createTimer(ros::Duration(2), compareCallback);

    marker_pub = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/inconsistencies_arrays", 10);

    ros::spin();
    return (0);
}
