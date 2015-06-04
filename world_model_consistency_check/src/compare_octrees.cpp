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
#include <colormap/colormap.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>

#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);

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

        double getSize(void) {return _max_x - _min_x;};
        double getVolume(void) {return getSize()*getSize()*getSize();};

        point3d getCenter(void) 
        {
            point3d p; 
            p.x() = (_max_x + _min_x)/2;
            p.y() = (_max_y + _min_y)/2;
            p.z() = (_max_z + _min_z)/2;
            return p;
        }


        std::vector<size_t> pointsInPointCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& pc)
        {

            std::vector<size_t> v;
            for (size_t i=0; i<pc->size(); ++i)
            {
                if (isPointInVolume(pc->points[i].x, pc->points[i].y, pc->points[i].z))
                {
                    v.push_back(i);
                }

            }
            return v;

        }

        bool isPointInVolume(float x, float y, float z)
        {
            if (x >= _min_x && x <= _max_x  &&
                    y >= _min_y && y <= _max_y &&
                    z >= _min_z && z <= _max_z) 
            {
                return true;
            }
            else
                return false;



        }

    protected:

        double _min_x;
        double _max_x;
        double _min_y;
        double _max_y;
        double _min_z;
        double _max_z;
};

// bool operator==(const ClassBoundingBox& lhs, const ClassBoundingBox& rhs) const
//         { 
//             if ( rhs.getMinimumPoint() == lhs.getMinimumPoint() )
//             {
//                 return true; 
//             }

//             return false;

//         }


bool are_neighbors(ClassBoundingBox b1, ClassBoundingBox b2)
{
    double cx1 = b1.getCenter().x();
    double cy1 = b1.getCenter().y();
    double cz1 = b1.getCenter().z();

    double cx2 = b2.getCenter().x();
    double cy2 = b2.getCenter().y();
    double cz2 = b2.getCenter().z();

    double dist = sqrt((cx1-cx2)*(cx1-cx2) + (cy1-cy2)*(cy1-cy2) + (cz1-cz2)*(cz1-cz2));

    double mx1 =  b1.getMinimumPoint().x();
    double my1 =  b1.getMinimumPoint().y();
    double mz1 =  b1.getMinimumPoint().z();
    double s1 = sqrt( (cx1 - mx1) * (cx1 - mx1) + (cy1 - my1) * (cy1 - my1) + (cz1 - mz1) * (cz1 - mz1));

    double mx2 =  b2.getMinimumPoint().x();
    double my2 =  b2.getMinimumPoint().y();
    double mz2 =  b2.getMinimumPoint().z();
    double s2 = sqrt( (cx2 - mx2) * (cx2 - mx2) + (cy2 - my2) * (cy2 - my2) + (cz2 - mz2) * (cz2 - mz2));



    //ROS_INFO("dx=%f dy=%f dz=%f", dx,dy,dz);
    //ROS_INFO("dist=%f", dist);
    //ROS_INFO("size_sum=%f", size_sum);
    //if (dist <= size_sum*50000000000)
    if (dist <= (s1 + s2))
        return true;
    else
        return false;
}



/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */


boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pc;
boost::shared_ptr<ros::Publisher> pub;
boost::shared_ptr<ros::Publisher> marker_pub;
boost::shared_ptr<ros::Publisher> marker_pub_center_of_mass;
boost::shared_ptr<ros::Publisher> marker_pub_inconsistencies;
boost::shared_ptr<ros::Publisher> marker_pub_clusters;
boost::shared_ptr<ros::Publisher> pub_pointcloud;
boost::shared_ptr<tf::TransformListener> listener;
boost::shared_ptr<ros::NodeHandle> nh;

OcTree* octree_model = NULL;
OcTree* octree_target = NULL;

std::string topic_model = "/octomap_model";
std::string topic_target = "/octomap_target";

unsigned char depth = 13;
double volume_threshold = 0.7;


//Declare a ClassBoundingBox which defines the target_volume
//ClassBoundingBox target_volume(-0.42, 0.42, -0.42, 0.42, 0.2, 3.0);
ClassBoundingBox target_volume(0.6, 1.4, -.7, .7, 0.6, 2.0);

std::string octree_frame_id = "kinect_rgb_optical_frame";






void octomapCallbackModel(const octomap_msgs::Octomap::ConstPtr& msg)
{
    AbstractOcTree* tree = msgToMap(*msg);
    octree_model = dynamic_cast<OcTree*>(tree);

    ROS_INFO("Received new octree model on topic %s with id: %s, frame_id: %s\n", topic_model.c_str(), msg->id.c_str(), msg->header.frame_id.c_str());

    // // int treeDepth = octree_model->getTreeDepth();
    // // ROS_INFO("treeDepth = %d", treeDepth); 
}

void octomapCallbackTarget(const octomap_msgs::Octomap::ConstPtr& msg)
{
    AbstractOcTree* tree = msgToMap(*msg);
    octree_target = dynamic_cast<OcTree*>(tree);

    ROS_INFO("Received new octree target on topic %s with id: %s, frame_id: %s\n", topic_target.c_str(), msg->id.c_str(), msg->header.frame_id.c_str());

    if (msg->header.frame_id != "")
        octree_frame_id = msg->header.frame_id;

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
    visualization_msgs::MarkerArray ma_inconsistencies;
    visualization_msgs::MarkerArray ma_clusters;
    unsigned int id=0;
    unsigned int id_inconsistencies=0;
    unsigned int id_noneighbors=0;
    unsigned int id_clusters=0;

    // Color initialization
    std_msgs::ColorRGBA color_occupied;
    color_occupied.r = 0; color_occupied.g = 0; color_occupied.b = 0.5; color_occupied.a = .8;

    std_msgs::ColorRGBA color_inconsistent;
    color_inconsistent.r = .5; color_inconsistent.g = 0; color_inconsistent.b = 0; color_inconsistent.a = .8;

    std_msgs::ColorRGBA color_target_volume;
    color_target_volume.r = .5; color_target_volume.g = 0.5; color_target_volume.b = 0; color_target_volume.a = 1;

    std_msgs::ColorRGBA color_noneighbors;
    color_noneighbors.r = .5; color_noneighbors.g = 0; color_noneighbors.b = 1; color_noneighbors.a = .8;


    //std_msgs::ColorRGBA color_cluster_green;
    //color_cluster_green.r = 0; color_cluster_green.g = 0.5; color_cluster_green.b = 0; color_cluster_green.a = .8;

    //std_msgs::ColorRGBA color_cluster_blue;
    //color_cluster_blue.r = 0; color_cluster_blue.g = 0; color_cluster_blue.b = 0.5; color_cluster_blue.a = .8;

    //std_msgs::ColorRGBA color_cluster_red;
    //color_cluster_red.r = 0.5; color_cluster_red.g = 0; color_cluster_red.b = 0; color_cluster_red.a = .8;

    //std_msgs::ColorRGBA color_cluster_yellow;
    //color_cluster_blue.r = 0; color_cluster_blue.g = 0.5; color_cluster_blue.b = 0.5; color_cluster_blue.a = .8;

    //std_msgs::ColorRGBA color_cluster_purple;
    //color_cluster_blue.r = 0.5; color_cluster_blue.g = 0; color_cluster_blue.b = 0.5; color_cluster_blue.a = .8;


    //// Vector of Cluster Colors initialization
    //std::vector<std_msgs::ColorRGBA> cluster_colors;
    //cluster_colors.push_back(color_cluster_green);
    //cluster_colors.push_back(color_cluster_blue);
    //cluster_colors.push_back(color_cluster_red);
    //cluster_colors.push_back(color_cluster_yellow);
    //cluster_colors.push_back(color_cluster_purple);


    // Vector of Inconsistencies initialization
    std::vector<ClassBoundingBox> vi;

    // Creates the target volume message array
    ma.markers.push_back(target_volume.getMarkerWithEdges("target_volume", octree_frame_id , color_target_volume, ++id));


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
                ma.markers.push_back(target_cell.getMarkerWithEdges("target_occupied", octree_frame_id , color_occupied, ++id));

                bool flg_found_occupied = false;
                bool flg_found_neighbors = false;
                int count =0;

                // -------------------------------------------------------------
                // ----------- Iterate over model octree ----------------------
                // -------------------------------------------------------------
                for(OcTree::leaf_bbx_iterator it_model = octree_model->begin_leafs_bbx(target_cell.getMinimumPoint(),target_cell.getMaximumPoint(), depth), end=octree_model->end_leafs_bbx(); it_model!= end; ++it_model)
                {
                    if (octree_model->search(it_model.getKey())) // Verifies if the nodes exists
                    {
                        flg_found_neighbors = true;

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

                if (flg_found_occupied == false && flg_found_neighbors == true) //If no occupied cell was found out of all iterated in the model's bbox, then an inconsistency is detected
                {
                    // Add the inconsistency cell into a vector
                    vi.push_back(target_cell);

                    ma_inconsistencies.markers.push_back(target_cell.getMarkerCubeVolume("target_inconsistent", octree_frame_id, color_inconsistent, ++id_inconsistencies));
                }
            }
        }
    }


    //Build the queue
    vector<size_t> queue;
    for (size_t i=0; i != vi.size(); ++i)
    {
        queue.push_back(i);
    }

    // //Print the queue list
    // for (size_t i=0; i != queue.size(); ++i)
    // {
    //     ROS_INFO("queue[%ld]=%ld", i, queue[i]);
    // }

    vector< vector<size_t> > cluster; 

    while (queue.size() != 0)
    {
        //Select new seed
        size_t seed = queue[0]; 
        queue.erase(queue.begin() + 0); //remove first element

        // ROS_INFO("Selected seed point %ld, queue has size=%ld", seed, queue.size());

        //Create new cluster
        vector<size_t> tmp;
        cluster.push_back(tmp);

        ROS_INFO("Created cluster %ld ", cluster.size());

        //Expand seed
        vector <size_t> flood;
        flood.push_back(seed);


        while (flood.size() != 0)
        {

            //ROS_INFO("Expanding first elem of flood (size %ld) idx = %ld", flood.size(), flood[0]);
            //expand flood[j]
            size_t idx_b1 = flood[0];


            //ROS_INFO("Checking of queue size %ld", queue.size());
            for (size_t j=0; j < queue.size(); ++j) 
            {
                size_t idx_b2 = queue[j]; 

                //ROS_INFO("Checking idx_b1 %ld idx_b2 %ld", idx_b1, idx_b2);


                //char name[50];
                //cout << "press a key to continue";
                //cin >> name;

                if (are_neighbors(vi[idx_b1], vi[idx_b2]))
                {
                    //ROS_INFO("Found neighbor idx %ld", idx_b2);
                    flood.push_back(idx_b2);
                    queue.erase(queue.begin() + j);
                    //TODO should be b2 or b1?

                }
                else
                {
                    //nothing to do 
                }
            }



            //add first elem of floodto cluster
            cluster.at(cluster.size()-1).push_back(flood[0]); //add seed point to cluster

            //remove first elem of  flood
            flood.erase(flood.begin() + 0);

        }


        ROS_INFO("Created cluster %ld with %ld points", cluster.size(), cluster[cluster.size()-1].size());



    }



    //char name[50];
    //cout << "press a key to continue";
    //cin >> name;


    ROS_INFO("There are %ld clusters", cluster.size());
    class_colormap cluster_colors("autumn", cluster.size(), 0.8);
    // ROS_INFO("Number of clusters found %ld", cluster.size());
    // for (size_t i=0; i < cluster.size(); ++i)
    // {
    //     ROS_INFO("Cluster %ld has the following points:", i);

    //     for (size_t j=0; j < cluster[i].size(); ++j)
    //     {
    //         cout << cluster[i][j] << ", "; 

    //     }

    //     cout << endl; 
    // }


    // ----------------------------------------------------
    // --------- Euclidean Cluster Extraction (RAFAEL) _---
    // ----------------------------------------------------


    // Create empty list of clusters
    // std::vector<std::vector<ClassBoundingBox *>> v_v_cluster;
    //std::vector< std::vector<ClassBoundingBox> > v_v_cluster; 

    //// Create queue of cells that need to be checked
    //std::vector<ClassBoundingBox> v_toCheck;


    //// For every cell in the dataset that is not part of a cluster already, 
    //// add the cell to the queue of cells that need to be checked.

    //// Initializes the Iterator to the dataset
    //for (std::vector<ClassBoundingBox>::iterator i = vi.begin(); i != vi.end(); i++)
    //{
    //// Tests if the cell is already part of a cluster.
    //if (true)
    //{
    //// If it is, jumps out of the cycle, because the cell is already in a cluster.
    //}

    //else
    //{
    //// Adds the cell to the vector of cells that need to be checked.
    //v_toCheck.push_back((*i));

    //// For every cell in the queue of cells that need to be checked:
    //for (std::vector<ClassBoundingBox>::iterator iq = v_toCheck.begin(); iq != v_toCheck.end(); iq++)
    //{
    //// Serch for cells in the Neighbourhood
    //// If there are cells in the Neighbourhood,
    //// Test if the are already not part of the queue of points that needs to be checked.

    //// if( std::find(v_toCheck.begin(), v_toCheck.end(), (*iq) ) != v_toCheck.end() ) 
    //// {
    ////     // v contains x 
    //// } 

    //// else 
    //// {
    ////     // v does not contain x
    //// }


    //// If they are not,  add then to the list of points that needs to be checked.

    //}

    //// When every point of the "to check" dataset is checked, add then to the cluster list and clean it.
    //// Draw the data cells to be visualized

    //}
    //}





    ROS_INFO("Inconsistencies vecotr has %ld cells", vi.size());

    ros::Duration d = (ros::Time::now() - t);
    ROS_INFO("Comparisson took %f secs", d.toSec());


/* _________________________________
  |                                       |
  |Filter clusters using volume threshold |
  |_________________________________      | */

    vector< vector<size_t> > selected_cluster; 

    for (size_t k = 0; k < cluster.size(); ++k)
    {
        //Assume all cells have the same volume
        double cell_volume = vi[cluster[k][0]].getVolume();
        double cluster_volume = cell_volume * cluster[k].size();

        if (cluster_volume > volume_threshold)
        {
            vector<size_t> tmp;
            // Iterates once per point of the cluster
            for (size_t l = 0; l < cluster[k].size(); ++l)
            {
                size_t cluster_aux = cluster[k][l];
                tmp.push_back(cluster[k][l]);
            }
            selected_cluster.push_back(tmp);
        }
    }
    ROS_INFO("Selected %ld clusters suing volume threshold", selected_cluster.size());



    // ----------------------------------------------------
    // --------- Draws clusters on visualizer -------------

    // Iterates once per cluster
    for (size_t k = 0; k < selected_cluster.size(); ++k)
    {
            // Iterates once per point of the cluster
            for (size_t l = 0; l < selected_cluster[k].size(); ++l)
            {
                size_t cluster_aux = selected_cluster[k][l];
                ma_clusters.markers.push_back(vi[cluster_aux].getMarkerCubeVolume("clusters", octree_frame_id, cluster_colors.color(k), ++id_clusters));
            }
    }


    // ----------------------------------------------------
    // ----------- Center of Mass of Clusters -------------
    // ----------------------------------------------------

    // Visualization Message Marker Array for the center of mass
    visualization_msgs::MarkerArray ma_centerofmass;
    int id_ma_centerofmass = 0;


    for (size_t m = 0; m < selected_cluster.size(); ++m)
    {

        double totalX = 0;
        double totalY = 0;
        double totalZ = 0;

        for (size_t n = 0; n < selected_cluster[m].size(); ++n)
        {

            size_t cluster_aux = selected_cluster[m][n];

            // double total_volume += vi[cluster_aux].getVolume();
            // double totalX += vi[cluster_aux].getCenter().x() * vi[cluster_aux].getVolume();

            // Calculate the sum of X
            totalX += vi[cluster_aux].getCenter().x();

            // Calculate the sum of Y
            totalY += vi[cluster_aux].getCenter().y();

            // Calculate the sum of Z
            totalZ += vi[cluster_aux].getCenter().z();
        }

        // Calculate the average of X
        double averageX = 0;
        averageX = totalX / selected_cluster[m].size();

        // Calculate the average of Y
        double averageY = 0;
        averageY = totalY / selected_cluster[m].size();

        // Calculate the average of Z
        double averageZ = 0;
        averageZ = totalZ / selected_cluster[m].size();

        ROS_INFO("Averages for Cluster[%ld]: X: %f, Y: %f, Z: %f", m, averageX, averageY, averageZ);

        visualization_msgs::Marker marker;
        marker.header.frame_id = octree_frame_id ;
        marker.header.stamp = ros::Time();
        marker.ns = "centerOfMass";
        marker.id = id_ma_centerofmass; //   ATENTION!!
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = averageX;
        marker.pose.position.y = averageY;
        marker.pose.position.z = averageZ;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        ma_centerofmass.markers.push_back(marker);

        id_ma_centerofmass++;

    }


    //Delete
    visualization_msgs::MarkerArray ma_deleteall;
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time();
    marker.header.frame_id = octree_frame_id ;
    marker.ns = "clusters";
    //marker.action = visualization_msgs::Marker::DELETEALL;
    marker.action = 3;
    ma_deleteall.markers.push_back(marker);
    marker_pub_clusters->publish(ma_deleteall);

    ma_deleteall.markers[0].ns = "target_inconsistent";
    marker_pub_inconsistencies->publish(ma_deleteall);

    marker_pub_inconsistencies->publish(ma_deleteall);


    marker_pub_inconsistencies->publish(ma_inconsistencies);
    marker_pub_clusters->publish(ma_clusters);

    //ros::Duration(0.05).sleep();

    marker_pub->publish(ma);

    marker_pub_center_of_mass->publish(ma_centerofmass);

    //publish colored point cloud
    cout << "Waiting for a point_cloud2 on topic " << "/camera/depth_registered/points" << endl;
    sensor_msgs::PointCloud2::ConstPtr pcmsg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", *nh, ros::Duration(0.5));

    ros::spinOnce();
    if (!pcmsg)
    {
        ROS_ERROR_STREAM("No point_cloud2 has been received after " << 0.5 << "secs");
    }
    else
    {
        ROS_INFO_STREAM("Received point cloud");

        pcl::fromROSMsg(*pcmsg, *pc);

        //bool    transformPointCloud (const std::string &target_frame, const pcl::PointCloud< PointT > &cloud_in, pcl::PointCloud< PointT > &cloud_out, const tf::TransformListener &tf_listener)
        //pcl_ros::transformPointCloud(octree_frame_id, pc, pc, listener);
        //pcl_ros::transformPointCloud<pcl::PointXYZRGB>(octree_frame_id, pc, pc, listener);
        pcl_ros::transformPointCloud(octree_frame_id, *pc, *pc, *listener);

        //pcl::transformPointCloud (*camera_ref_system, *shelf_ref_system, t); 
        //
        //
        //
        //ROS_ERROR("There are %ld points the point cloud", pc->size());

        for (size_t k = 0; k < selected_cluster.size(); ++k)
        {
            std::vector<size_t> lpoints;
            for (size_t l = 0; l < selected_cluster[k].size(); ++l)
            {
                size_t idx = selected_cluster[k][l];
                std::vector<size_t> ltmp;
                ltmp = vi[idx].pointsInPointCloud(pc);
                //ROS_ERROR("There are %ld points in cube %ld of cluster %ld", lpoints.size(),l, k);
                lpoints.insert(lpoints.end(), ltmp.begin(), ltmp.end());
            }

            //ROS_ERROR("There are %ld points in cluster %ld", lpoints.size(), k);
            //change color of points to cluster color
            for (size_t i=0; i< lpoints.size(); ++i)
            {
                cv::Scalar c = cluster_colors.cv_color(k);
                int8_t r = c[2], g = c[1], b = c[0];
                //int8_t r = 255, g = 0, b = 0;    // Example: Red color
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                //p.rgb = *reinterpret_cast<float*>(&rgb);
                pc->points[lpoints[i]].rgb = *reinterpret_cast<float*>(&rgb);;

            }
        }

        sensor_msgs::PointCloud2 pcmsgout;
        pcl::toROSMsg(*pc, pcmsgout);
        pub_pointcloud->publish(pcmsgout);

    }


}


void callbackDynamicReconfigure(world_model_consistency_check::DepthConfigurationConfig &config, uint32_t level) 
{
    ROS_INFO("Reconfigure Request: Setting comparison depth to %d",  config.depth);
    ROS_INFO("Reconfigure Request: Setting volume threshold to %f",  config.volume_threshold);
    depth = (unsigned char) config.depth;
    volume_threshold = (double) config.volume_threshold;
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "compare_octrees");
    nh = (boost::shared_ptr<ros::NodeHandle>) new ros::NodeHandle;

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


    listener = (boost::shared_ptr<tf::TransformListener>) new (tf::TransformListener);
    ros::Duration(1).sleep(); // sleep for a second


    pc = (pcl::PointCloud<pcl::PointXYZRGB>::Ptr) new (pcl::PointCloud<pcl::PointXYZRGB>);

    ros::Subscriber sub_model = nh->subscribe(topic_model, 0, octomapCallbackModel);
    ros::Subscriber sub_target = nh->subscribe(topic_target, 0, octomapCallbackTarget);

    ros::Timer timer = nh->createTimer(ros::Duration(2), compareCallback);

    marker_pub = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub = nh->advertise<visualization_msgs::MarkerArray>("/target_volume", 10);

    marker_pub_center_of_mass = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub_center_of_mass = nh->advertise<visualization_msgs::MarkerArray>("/center_of_mass", 10);

    marker_pub_inconsistencies = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub_inconsistencies = nh->advertise<visualization_msgs::MarkerArray>("/inconsistencies", 10);

    marker_pub_clusters = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub_clusters = nh->advertise<visualization_msgs::MarkerArray>("/clusters", 10);

    pub_pointcloud = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *pub_pointcloud = nh->advertise<sensor_msgs::PointCloud2>("/inconsistent_points", 0);




    ros::spin();
    return (0);
}
