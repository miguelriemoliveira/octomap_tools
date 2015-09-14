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

#include <dynamic_reconfigure/server.h>
#include <world_model_consistency_check/DepthConfigurationConfig.h>
#include <colormap/colormap.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>

// My includes
#include <world_model_consistency_check/bounding_box.h>

#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);

using namespace std;
using namespace octomap;
using namespace octomap_msgs;
using namespace sensor_msgs;


/* _________________________________
  |                                 |
  |       FUNCTION PROTOTYPES       |
  |_________________________________| */


bool are_neighbors(ClassBoundingBox b1, ClassBoundingBox b2);


/* _________________________________
  |                                 |
  |      FUNCTION DEFINITIONS       |
  |_________________________________| */


void centerOfMass(vector<ClassBoundingBox>& vi, vector< vector<size_t> >& cluster, visualization_msgs::MarkerArray& ma, visualization_msgs::MarkerArray& ma_volumeText, size_t &id, string frame_id)
{
// Function to compute the Center of Mass of a Cluster.

    // For each Cluster
    for (size_t m = 0; m < cluster.size(); ++m)
    {
        double cluster_volume = 0;
        double totalX = 0;
        double totalY = 0;
        double totalZ = 0;

        double cell_volume = 0;

        // Iterate each cell inside a given Cluster
        for (size_t n = 0; n < cluster[m].size(); ++n)
        {

            size_t cluster_aux = cluster[m][n];

            // Calculate the sum of X
            totalX += vi[cluster_aux].getCenter().x();

            // Calculate the sum of Y
            totalY += vi[cluster_aux].getCenter().y();

            // Calculate the sum of Z
            totalZ += vi[cluster_aux].getCenter().z();

            // Cell Volume
            cell_volume = vi[cluster[m][n]].getVolume();
        }

        // Calculate the average of X
        double averageX = 0;
        averageX = totalX / cluster[m].size();

        // Calculate the average of Y
        double averageY = 0;
        averageY = totalY / cluster[m].size();

        // Calculate the average of Z
        double averageZ = 0;
        averageZ = totalZ / cluster[m].size();

        ROS_INFO("Averages for Cluster [%ld]: X: %f, Y: %f, Z: %f", m, averageX, averageY, averageZ);

        // Marker Array of Center of Mass
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id ;
        marker.header.stamp = ros::Time();
        marker.ns = "centerOfMass";
        marker.id = id; //   ATENTION!!
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(1.9);

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
        marker.color.r = 0.4;
        marker.color.g = 0.4;
        marker.color.b = 0.4;

        ma.markers.push_back(marker);


        // Cluster Volume is the sum of each cell
        cluster_volume += cell_volume;


        // Marker Array for Volume 
        visualization_msgs::Marker marker_volume;
        marker_volume.header.frame_id = frame_id ;
        marker_volume.header.stamp = ros::Time();
        marker_volume.ns = "volume";
        marker_volume.id = id; //   ATENTION!!
        marker_volume.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_volume.action = visualization_msgs::Marker::ADD;

        marker_volume.pose.position.x = averageX;
        marker_volume.pose.position.y = averageY;
        marker_volume.pose.position.z = averageZ;

        // Convertion double to string
        std::ostringstream os;
        char ss[1024];

        sprintf(ss, "%0.3f", cluster_volume);
        std::string str_volume = ss;

        sprintf(ss, "%0.2f", averageX);
        std::string str_averageX = ss;

        sprintf(ss, "%0.2f", averageY);
        std::string str_averageY = ss;

        sprintf(ss, "%0.2f", averageZ);
        std::string str_averageZ = ss;

        marker_volume.text = std::string("X: ") + str_averageX + std::string(" Y: ") + str_averageY + std::string(" Z: ") + str_averageZ + "\n" + std::string("Volume: ") + str_volume + std::string("m^3");

        marker_volume.scale.z = 0.2; // Size of Text
        marker_volume.color.a = 1;
        marker_volume.color.r = 0;
        marker_volume.color.g = 0;
        marker_volume.color.b = 1;

        marker_volume.lifetime = ros::Duration(1.9);

        ma_volumeText.markers.push_back(marker_volume);

        id++;

    }
}

void clustersToMarkerArray(vector<ClassBoundingBox>& vi, vector< vector<size_t> >& cluster, visualization_msgs::MarkerArray& ma, size_t& id, string frame_id, string ns, class_colormap& cluster_colors)
{
// Function to create Marker Array to graphically represent a given Cluster.

    // Iterates once per cluster
    for (size_t k = 0; k < cluster.size(); ++k)
    {
        // Iterates once per cell of the cluster
        for (size_t l = 0; l < cluster[k].size(); ++l)
        {
            size_t cluster_aux = cluster[k][l];
            ma.markers.push_back(vi[cluster_aux].getMarkerCubeVolume(ns, frame_id, cluster_colors.color(k), ++id));
        }
    }
}

void filterClustersByVolume(vector<ClassBoundingBox>& vi, vector< vector<size_t> >& cluster, vector< vector<size_t> >& selected_cluster, double volume_threshold)
{
// Function to filter Clusters by a given Volume Threshold.

    // Iterates once per Cluster
    for (size_t k = 0; k < cluster.size(); ++k)
    {
        //Assume all cells have the same volume
        double cell_volume = ((ClassBoundingBox) vi[cluster[k][0]]).getVolume();

        // Cluster Volume calculation
        double cluster_volume = cell_volume * cluster[k].size();

        // Filtering by a given Volume Threshold
        if (cluster_volume > volume_threshold)
        {
            vector<size_t> tmp;

            // Iterates once per cell of the Cluster
            for (size_t l = 0; l < cluster[k].size(); ++l)
            {
                tmp.push_back(cluster[k][l]);
            }

            // Stores the Filtered Clusters
            selected_cluster.push_back(tmp);
        }
    }
}

void clusterBoundingBoxes(vector<ClassBoundingBox>& vi, vector< vector<size_t> >& cluster)
{
// Function to create Clusters by a flood fill algorithm.

    //Build the queue
    vector<size_t> queue;
    for (size_t i=0; i != vi.size(); ++i)
    {
        queue.push_back(i);
    }

    while (queue.size() != 0)
    {
        //Select new seed
        size_t seed = queue[0]; 
        queue.erase(queue.begin() + 0); //remove first element

        //Create new cluster
        vector<size_t> tmp;
        cluster.push_back(tmp);

        //Expand seed
        vector <size_t> flood;
        flood.push_back(seed);

        while (flood.size() != 0)
        {

            size_t idx_b1 = flood[0];

            // Iterates over every cell still on the queue
            for (size_t j=0; j < queue.size(); ++j) 
            {
                size_t idx_b2 = queue[j]; 

                // Check if elements are neighbors
                if (are_neighbors(vi[idx_b1], vi[idx_b2]))
                {
                    flood.push_back(idx_b2);
                    queue.erase(queue.begin() + j);
                }
                else
                {
                    // do nothing
                }
            }

            // Add first element of Flood to Cluster
            cluster.at(cluster.size()-1).push_back(flood[0]);

            // Remove first element of Flood
            flood.erase(flood.begin() + 0);

        }
    }
}

bool are_neighbors(ClassBoundingBox b1, ClassBoundingBox b2)
{
// Function to check if two cells are neighbors.

// The neighborhood criteria for considereing two cells neighbors is defined as the sum of the distances
// between the extremities of the cell and the centers has to be lower or equal (given a fitting threshold)
// to the distance between centers of both cells

    // Center of First Cell along the X, Y and Z coordinates
    double cx1 = b1.getCenter().x();
    double cy1 = b1.getCenter().y();
    double cz1 = b1.getCenter().z();

    // Center of Second Cell along the X, Y and Z coordinates
    double cx2 = b2.getCenter().x();
    double cy2 = b2.getCenter().y();
    double cz2 = b2.getCenter().z();

    // Minimum points of First Cell along the X, Y and Z coordinates
    double mx1 =  b1.getMinimumPoint().x();
    double my1 =  b1.getMinimumPoint().y();
    double mz1 =  b1.getMinimumPoint().z();

    // Minimum points of Second Cell along the X, Y and Z coordinates
    double mx2 =  b2.getMinimumPoint().x();
    double my2 =  b2.getMinimumPoint().y();
    double mz2 =  b2.getMinimumPoint().z();
    
    // Distances between centers of both cells along the X, Y and Z coordinates
    double dist_cx = sqrt((cx1-cx2)*(cx1-cx2));
    double dist_cy = sqrt((cy1-cy2)*(cy1-cy2));
    double dist_cz = sqrt((cz1-cz2)*(cz1-cz2));

    // Sum of distances between centers of both cells and the their extremeties.
    double dist_mx = abs(cx1 - mx1) + abs(cx2 - mx2);
    double dist_my = abs(cy1 - my1) + abs(cy2 - my2);
    double dist_mz = abs(cz1 - mz1) + abs(cz2 - mz2);

    // Neighborhood Criteria
    if (dist_cx <= dist_mx*1.05 && dist_cy <= dist_my*1.05 && dist_cz <= dist_mz*1.05)
        return true;
    else
        return false;

}


/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */


boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pc;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pc2;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pcin;
boost::shared_ptr<ros::Publisher> pub;
boost::shared_ptr<ros::Publisher> marker_pub;
boost::shared_ptr<ros::Publisher> marker_pub_center_of_mass;
boost::shared_ptr<ros::Publisher> marker_pub_volume;
boost::shared_ptr<ros::Publisher> marker_pub_inconsistencies;
boost::shared_ptr<ros::Publisher> marker_pub_clusters;
boost::shared_ptr<ros::Publisher> pub_pointcloud;
boost::shared_ptr<tf::TransformListener> listener;
boost::shared_ptr<ros::NodeHandle> nh;

OcTree* octree_model = NULL;
OcTree* octree_target = NULL;

std::string topic_model = "/octomap_model";
std::string topic_target = "/octomap_target";
std::string topic_point_cloud = "/camera/depth_registered/points";
bool use_regions = false;
bool permanent_markers = false;

unsigned char depth = 13;
double volume_threshold = 0.7;
double exceeding_threshold = 0.2;
double missing_threshold = 0.5;
double exceeding_threshold_with_regions = 0.1;
double missing_threshold_with_regions = 0.9;

//Declare a ClassBoundingBox which defines the target_volume
ClassBoundingBox target_volume(0.4, 2.0, -1.0, 1., 0.3, 2.2);

std::string octree_frame_id = "world";
bool flg_received_new_target = false;
bool flg_received_point_cloud = false;
AbstractOcTree* model_tree = NULL;
AbstractOcTree* target_tree = NULL;
std::vector<ClassBoundingBox> boxes;

/* _________________________________
   |                                 |
   |           Callbacks             |
   |_________________________________| */

void load_regions(void)
{
// Callback to load previously saved regions.

    // Retrieves the vector of center positions of regions from topic.
    std::vector<double> center_x;
    std::vector<double> center_y;
    std::vector<double> center_z;
    ros::param::get("/interactive_region_definition/center_x", center_x);
    ros::param::get("/interactive_region_definition/center_y", center_y);
    ros::param::get("/interactive_region_definition/center_z", center_z);

    // Retrieves the vector of sizes of regions from topic.
    std::vector<double> size_x;
    std::vector<double> size_y;
    std::vector<double> size_z;
    ros::param::get("/interactive_region_definition/size_x", size_x);
    ros::param::get("/interactive_region_definition/size_y", size_y);
    ros::param::get("/interactive_region_definition/size_z", size_z);

    // Retrieves the vector of occupation information from topic.
    std::vector<bool> is_occupied;
    ros::param::get("/interactive_region_definition/is_occupied", is_occupied);

    // Sets the defauld frame_id and gets the frame_id from topic.
    std::string frame_id = "map";
    ros::param::get("/interactive_region_definition/frame_id", frame_id);

    ROS_INFO("There are %ld loaded boxes", center_x.size());

    // Erase previoysly stored boxes vector.
    boxes.erase(boxes.begin(), boxes.end());

    ROS_INFO("There are %ld boxes in memory", boxes.size());
    
    // Iterates over each region retrieved and stores its information on a vector.
    for (size_t i=0; i< center_x.size(); ++i)
    {
        ClassBoundingBox b(center_x[i] - size_x[i]/2, center_x[i] + size_x[i]/2, center_y[i] - size_y[i]/2, center_y[i] + size_y[i]/2, center_z[i] - size_z[i]/2, center_z[i] + size_z[i]/2);
        b.occupied = is_occupied[i];
        boxes.push_back(b);
    }

    ROS_INFO("Now there are %ld boxes in memory", boxes.size());
}

void octomapCallbackModel(const octomap_msgs::Octomap::ConstPtr& msg)
{
// Callback responsible for retrieving the Model Octomap from a topic and storing it in a variable.

    // Memory Leak fix
    if (model_tree != NULL)
    {
        delete(model_tree);
    }

    model_tree = msgToMap(*msg);
    octree_model = dynamic_cast<OcTree*>(model_tree);
}

void octomapCallbackTarget(const octomap_msgs::Octomap::ConstPtr& msg)
{
// Callback responsible for retrieving the Target Octomap from a topic and storing it in a variable.

    // Memory Leak fix
    if (target_tree != NULL)
    {
        delete(target_tree);
    }

    target_tree = msgToMap(*msg);
    octree_target = dynamic_cast<OcTree*>(target_tree);

    if (msg->header.frame_id != "")
        octree_frame_id = msg->header.frame_id;

    flg_received_new_target = true;
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
// Callback responsible for retrieving a Point Cloud from a topic and storing in in a variable.

    pcl::fromROSMsg(*msg, *pcin);
    flg_received_point_cloud = true;
}

void compareCallback(const ros::TimerEvent&)
{
// MODE 1: COMPARE TWO OCTOMAPS
// Compare two OctoMaps retrieved by topics and check for inconsistencies.

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

    // Checks if the received target has changed, if not, exits the callback.
    if (flg_received_new_target==true)
    {
        flg_received_new_target = false;
    }
    else
    {
        return;
    }

    // Visualization Message Marker Array Initialization
    visualization_msgs::MarkerArray ma;
    visualization_msgs::MarkerArray ma_inconsistencies;
    visualization_msgs::MarkerArray ma_clusters;

    visualization_msgs::Marker marker_deleteall;
    marker_deleteall.header.stamp = ros::Time();
    marker_deleteall.header.frame_id = octree_frame_id ;
    marker_deleteall.ns = "target_inconsistent";
    marker_deleteall.action = 3;

    size_t id=0;
    size_t id_inconsistencies=0;
    size_t id_noneighbors=0;
    size_t id_clusters=0;

    // Color initialization
    std_msgs::ColorRGBA color_occupied;
    color_occupied.r = 0; color_occupied.g = 0; color_occupied.b = 0.5; color_occupied.a = .8;

    std_msgs::ColorRGBA color_inconsistent;
    color_inconsistent.r = .5; color_inconsistent.g = 0; color_inconsistent.b = 0; color_inconsistent.a = .8;
    std_msgs::ColorRGBA color_inconsistent_missing;
    color_inconsistent_missing.r = .0; color_inconsistent_missing.g = 0.5; color_inconsistent_missing.b = 0; color_inconsistent_missing.a = .8;

    std_msgs::ColorRGBA color_target_volume;
    color_target_volume.r = .5; color_target_volume.g = 0.5; color_target_volume.b = 0; color_target_volume.a = 1;

    std_msgs::ColorRGBA color_noneighbors;
    color_noneighbors.r = .5; color_noneighbors.g = 0; color_noneighbors.b = 1; color_noneighbors.a = .8;


    // Vector of Inconsistencies initialization
    std::vector<ClassBoundingBox> vi;
    std::vector<ClassBoundingBox> vi_missing;

    // Creates the target volume message array
    ma.markers.push_back(target_volume.getMarkerWithEdges("target_volume", octree_frame_id , color_target_volume, ++id));


    ROS_INFO_STREAM("Starting Iteration");

    // Iterates over TARGET Octree
    for(OcTree::leaf_bbx_iterator it = octree_target->begin_leafs_bbx(target_volume.getMinimumPoint(), target_volume.getMaximumPoint(), depth), end=octree_target->end_leafs_bbx(); it!= end; ++it)
    {
        // Verifies if the cell exists
        if (octree_target->search(it.getKey())) 
        {
            // Verifies if the cell is occupied
            if (octree_target->isNodeOccupied(*it)) 
            {
                ClassBoundingBox target_cell(it.getX(), it.getY(), it.getZ(), it.getSize());
                
                size_t num_occupied = 0;
                size_t num_neighbors = 0;

                ma.markers.push_back(target_cell.getMarkerWithEdges("target_occupied", octree_frame_id , color_occupied, ++id));

                // Iterates over MODEL Octree             
                for(OcTree::leaf_bbx_iterator it_model = octree_model->begin_leafs_bbx(target_cell.getMinimumPoint(),target_cell.getMaximumPoint(), depth), end=octree_model->end_leafs_bbx(); it_model!= end; ++it_model)
                {
                    // Verifies if the cell exists
                    if (octree_model->search(it_model.getKey())) 
                    {
                        num_neighbors++;

                        // Verifies if the cell is FREE
                        if (!octree_model->isNodeOccupied(*it_model)) 
                        {
                            // Do Nothing
                        }
                        else
                        {
                            num_occupied++;
                        }
                    }
                }

                // Occupation Ratio computation
                double occupation_ratio=0;
                if (num_neighbors !=0)
                {
                    occupation_ratio = (double)num_occupied/(double)num_neighbors;
                }

                // Checks for Inconsistencies of type EXCEEDING
                // If no occupied cell was found out of all iterated in the model's bbox, then an inconsistency is detected
                if (occupation_ratio <= exceeding_threshold && num_neighbors !=0) 
                {
                    // Add the inconsistency cell into a vector
                    vi.push_back(target_cell);

                    ma_inconsistencies.markers.push_back(target_cell.getMarkerCubeVolume("target_inconsistent", octree_frame_id, color_inconsistent, ++id_inconsistencies,permanent_markers));
                }
            }
        }
    }



    // Iterates over MODEL Octree
    for(OcTree::leaf_bbx_iterator it = octree_model->begin_leafs_bbx(target_volume.getMinimumPoint(), target_volume.getMaximumPoint(), depth), end=octree_model->end_leafs_bbx(); it!= end; ++it)
    {
        // Verifies if the cell exists
        if (octree_model->search(it.getKey())) 
        {
            // Verifies if cell is Occupied
            if (octree_model->isNodeOccupied(*it))
            {
                ClassBoundingBox model_cell(it.getX(), it.getY(), it.getZ(), it.getSize());

                size_t num_occupied = 0;
                size_t num_neighbors = 0;

                ma.markers.push_back(model_cell.getMarkerWithEdges("model_occupied", octree_frame_id , color_occupied, ++id));

                // Iterates over TARGET Octree
                for(OcTree::leaf_bbx_iterator it_target = octree_target->begin_leafs_bbx(model_cell.getMinimumPoint(),model_cell.getMaximumPoint(), depth), end=octree_target->end_leafs_bbx(); it_target!= end; ++it_target)
                {
                    // Verifies if cell exists
                    if (octree_target->search(it_target.getKey())) 
                    {
                        num_neighbors++;

                        // Verifies if cell is FREE
                        if (!octree_target->isNodeOccupied(*it_target)) 
                        {
                            // Do nothing
                        }
                        else
                        {
                            num_occupied++;
                        }
                    }
                }

                // Occupation Ratio computation
                double occupation_ratio=0;
                if (num_neighbors !=0)
                {
                    occupation_ratio = (double)num_occupied/(double)num_neighbors;
                }

                // Checks for Inconsistencies of type EXCEEDING
                // If no occupied cell was found out of all iterated in the model's bbox, then an inconsistency is detected
                if (occupation_ratio <= missing_threshold && num_neighbors !=0) 
                {
                    // Add the inconsistency cell into a vector
                    vi_missing.push_back(model_cell);

                    ma_inconsistencies.markers.push_back(model_cell.getMarkerCubeVolume("target_inconsistent", octree_frame_id, color_inconsistent_missing, ++id_inconsistencies,permanent_markers));
                }
            }
        }
    }



    /* _________________________________
       |                                 |
       |     for the EXCEEDING clusters  |
       |_________________________________| */


    //Build the queue
    vector<size_t> queue;
    for (size_t i=0; i != vi.size(); ++i)
    {
        queue.push_back(i);
    }

    vector< vector<size_t> > cluster; 

    while (queue.size() != 0)
    {
        //Select new seed
        size_t seed = queue[0]; 
        queue.erase(queue.begin() + 0); //remove first element

        //Create new cluster
        vector<size_t> tmp;
        cluster.push_back(tmp);

        //Expand seed
        vector <size_t> flood;
        flood.push_back(seed);

        while (flood.size() != 0)
        {

            size_t idx_b1 = flood[0];

            // Iterates over every cell still on the queue
            for (size_t j=0; j < queue.size(); ++j) 
            {
                size_t idx_b2 = queue[j]; 

                // Check if elements are neighbors
                if (are_neighbors(vi[idx_b1], vi[idx_b2]))
                {
                    flood.push_back(idx_b2);
                    queue.erase(queue.begin() + j);

                }
                else
                {
                    // Do nothing
                }
            }

            // Add first element of Flood to Cluster
            cluster.at(cluster.size()-1).push_back(flood[0]); //add seed point to cluster

            // Remove first element of Flood to Cluster
            flood.erase(flood.begin() + 0);
        }
    }


    /* _________________________________
       |                                 |
       |     for the missing clusters    |
       |_________________________________| */


    //Build the queue
    vector<size_t> queue_missing;
    for (size_t i=0; i != vi_missing.size(); ++i)
    {
        queue_missing.push_back(i);
    }

    vector< vector<size_t> > cluster_missing; 

    while (queue_missing.size() != 0)
    {
        //Select new seed
        size_t seed = queue_missing[0]; 
        queue_missing.erase(queue_missing.begin() + 0); //remove first element

        //Create new cluster
        vector<size_t> tmp;
        cluster_missing.push_back(tmp);

        //Expand seed
        vector <size_t> flood;
        flood.push_back(seed);

        while (flood.size() != 0)
        {
            size_t idx_b1 = flood[0];

            // Iterates over every cell still on the queue
            for (size_t j=0; j < queue_missing.size(); ++j) 
            {
                size_t idx_b2 = queue_missing[j]; 

                // Check if elements are neighbors
                if (are_neighbors(vi_missing[idx_b1], vi_missing[idx_b2]))
                {
                    flood.push_back(idx_b2);
                    queue_missing.erase(queue_missing.begin() + j);
                }
                else
                {
                    // do nothing
                }
            }

            // Add first element of Flood to Cluster
            cluster_missing.at(cluster_missing.size()-1).push_back(flood[0]); //add seed point to cluster

            // Remove first element of Flood
            flood.erase(flood.begin() + 0);
        }
    }

    //Information about clusters
    ROS_INFO("There are %ld clusters", cluster.size());
    ROS_INFO("There are %ld clusters_missing", cluster_missing.size());


    /* ______________________________________
       |                                      |
       |    Exceeding Clusters                |
       |________________________________      | */

    //Filter clusters using volume threshold

    vector< vector<size_t> > selected_cluster; 

    // Iterates once per Cluster
    for (size_t k = 0; k < cluster.size(); ++k)
    {
        //Assume all cells have the same volume
        double cell_volume = vi[cluster[k][0]].getVolume();
        
        // Cluster Volume calculation
        double cluster_volume = cell_volume * cluster[k].size();

        // Filtering by a given Volume Threshold
        if (cluster_volume > volume_threshold)
        {
            vector<size_t> tmp;

            // Iterates once per point of the cluster
            for (size_t l = 0; l < cluster[k].size(); ++l)
            {
                size_t cluster_aux = cluster[k][l];
                tmp.push_back(cluster[k][l]);
            }

            // Stores the Filtered Cluster
            selected_cluster.push_back(tmp);
        }
    }


    class_colormap cluster_colors("autumn", selected_cluster.size(), 0.8);

    // Draws cluster on visualizer
    // Iterates once per cluster
    for (size_t k = 0; k < selected_cluster.size(); ++k)
    {
        // Iterates once per point of the cluster
        for (size_t l = 0; l < selected_cluster[k].size(); ++l)
        {
            size_t cluster_aux = selected_cluster[k][l];
            ma_clusters.markers.push_back(vi[cluster_aux].getMarkerCubeVolume("clusters", octree_frame_id, cluster_colors.color(k), ++id_clusters, permanent_markers));
        }
    }


    /* ______________________________________
       | 
       |    Missing Clusters                  |
       |________________________________      | */

    //Filter clusters using volume threshold |

    vector< vector<size_t> > selected_cluster_missing; 

    // Iterates once per Cluster
    for (size_t k = 0; k < cluster_missing.size(); ++k)
    {
        //Assume all cells have the same volume
        double cell_volume = vi_missing[cluster_missing[k][0]].getVolume();
        
        // Cluster Volume calculation
        double cluster_volume = cell_volume * cluster_missing[k].size();

        // Filtering by a given Volume Threshold
        if (cluster_volume > volume_threshold)
        {
            vector<size_t> tmp;

            // Iterates once per point of the cluster
            for (size_t l = 0; l < cluster_missing[k].size(); ++l)
            {
                size_t cluster_aux = cluster_missing[k][l];
                tmp.push_back(cluster_missing[k][l]);
            }

            // Stores the Filtered Clusters
            selected_cluster_missing.push_back(tmp);
        }
    }

    class_colormap cluster_missing_colors("winter", selected_cluster_missing.size(), 0.8, true);

    // Draws cluster on visualizer
    // Iterates once per cluster
    for (size_t k = 0; k < selected_cluster_missing.size(); ++k)
    {
        // Iterates once per point of the cluster_missing
        for (size_t l = 0; l < selected_cluster_missing[k].size(); ++l)
        {
            size_t cluster_aux = selected_cluster_missing[k][l];
            ma_clusters.markers.push_back(vi_missing[cluster_aux].getMarkerCubeVolume("clusters", octree_frame_id, cluster_missing_colors.color(k), ++id_clusters, permanent_markers));
        }
    }





    /* ______________________________________
       |                                      |
       |    Exceeding Clusters                |
       |________________________________      | */

    // Center of Mass of Clusters

    // Visualization Message Marker Array for the center of mass
    visualization_msgs::MarkerArray ma_centerofmass;
    int id_ma_centerofmass = 0;

    // For each Cluster
    for (size_t m = 0; m < selected_cluster.size(); ++m)
    {

        double totalX = 0;
        double totalY = 0;
        double totalZ = 0;

        // Iterate each cell inside a given Cluster
        for (size_t n = 0; n < selected_cluster[m].size(); ++n)
        {

            size_t cluster_aux = selected_cluster[m][n];

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

        ROS_INFO("Averages for Cluster Exceeding[%ld]: X: %f, Y: %f, Z: %f", m, averageX, averageY, averageZ);

        visualization_msgs::Marker marker;
        marker.header.frame_id = octree_frame_id ;
        marker.header.stamp = ros::Time();
        marker.ns = "centerOfMass";
        marker.id = id_ma_centerofmass; //   ATENTION!!
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(1.9);

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
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        ma_centerofmass.markers.push_back(marker);

        id_ma_centerofmass++;
    }


    // Print Volume of Clusters
    
    // Visualization Message Marker Array for the center of mass
    visualization_msgs::MarkerArray ma_volumeText;
    int id_ma_volume = 0;

    // For each Cluster
    for (size_t m = 0; m < selected_cluster.size(); ++m)
    {

        double totalX = 0;
        double totalY = 0;
        double totalZ = 0;

        // Iterates each cell inside a given Cluster
        for (size_t n = 0; n < selected_cluster[m].size(); ++n)
        {

            size_t cluster_aux = selected_cluster[m][n];

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

        // Compute the volume
        // Assume all cells have the same volume
        double cell_volume = vi[selected_cluster[m][0]].getVolume();
        double cluster_volume = cell_volume * selected_cluster[m].size();

        // Create the marker
        visualization_msgs::Marker marker_volume;
        marker_volume.header.frame_id = octree_frame_id ;
        marker_volume.header.stamp = ros::Time();
        marker_volume.ns = "volume";
        marker_volume.id = id_ma_volume; //   ATENTION!!
        marker_volume.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_volume.action = visualization_msgs::Marker::ADD;

        marker_volume.pose.position.x = averageX;
        marker_volume.pose.position.y = averageY;
        marker_volume.pose.position.z = averageZ;

        // Convertion double to string
        std::ostringstream os;
        char ss[1024];

        sprintf(ss, "%0.6f", cluster_volume);
        std::string str_volume = ss;

        sprintf(ss, "%0.3f", averageX);
        std::string str_averageX = ss;

        sprintf(ss, "%0.3f", averageY);
        std::string str_averageY = ss;

        sprintf(ss, "%0.3f", averageZ);
        std::string str_averageZ = ss;

        marker_volume.text = std::string("X: ") + str_averageX + std::string(" Y: ") + str_averageY + std::string(" Z: ") + str_averageZ + "\n" + std::string("Volume: ") + str_volume;

        marker_volume.scale.z = 0.05; // Size of Text
        marker_volume.color.a = 1;
        marker_volume.color.r = 1;
        marker_volume.color.g = 1;
        marker_volume.color.b = 1;

        marker_volume.lifetime = ros::Duration(1.9);

        ma_volumeText.markers.push_back(marker_volume);

        id_ma_volume++;
    }



    /* ______________________________________
       |                                      |
       |    Missing Clusters                  |
       |________________________________      | */


    // Center of Mass of Clusters

    // For each Cluster
    for (size_t m = 0; m < selected_cluster_missing.size(); ++m)
    {

        double totalX = 0;
        double totalY = 0;
        double totalZ = 0;

        // Iterates each cell inside a given Cluster
        for (size_t n = 0; n < selected_cluster_missing[m].size(); ++n)
        {

            size_t cluster_aux = selected_cluster_missing[m][n];

            // Calculate the sum of X
            totalX += vi_missing[cluster_aux].getCenter().x();

            // Calculate the sum of Y
            totalY += vi_missing[cluster_aux].getCenter().y();

            // Calculate the sum of Z
            totalZ += vi_missing[cluster_aux].getCenter().z();
        }

        // Calculate the average of X
        double averageX = 0;
        averageX = totalX / selected_cluster_missing[m].size();

        // Calculate the average of Y
        double averageY = 0;
        averageY = totalY / selected_cluster_missing[m].size();

        // Calculate the average of Z
        double averageZ = 0;
        averageZ = totalZ / selected_cluster_missing[m].size();

        ROS_INFO("Averages for Cluster Missing[%ld]: X: %f, Y: %f, Z: %f", m, averageX, averageY, averageZ);

        visualization_msgs::Marker marker;
        marker.header.frame_id = octree_frame_id ;
        marker.header.stamp = ros::Time();
        marker.ns = "centerOfMass";
        marker.id = id_ma_centerofmass; //   ATENTION!!
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(1.9);

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

    // ???Volume Calculation???

    // Publish the Marker Arrays
    marker_pub_inconsistencies->publish(ma_inconsistencies);
    marker_pub_clusters->publish(ma_clusters);
    marker_pub->publish(ma);
    marker_pub_center_of_mass->publish(ma_centerofmass);
    marker_pub_volume->publish(ma_volumeText);

    // Paint the Point Cloud (if available) with the Inconsistencies information.
    if (!flg_received_point_cloud)
    {
        ROS_ERROR_STREAM("No point_cloud2 has been received yet");
    }
    else
    {
        ROS_INFO("Processing point cloud ...");
        *pc = *pcin;
        pcl_ros::transformPointCloud(octree_frame_id, *pc, *pc, *listener);
        *pc2 = *pc;
        pc2->points.erase(pc2->points.begin(), pc2->points.end());

        for (size_t k = 0; k < selected_cluster.size(); ++k)
        {
            std::vector<size_t> lpoints;
            for (size_t l = 0; l < selected_cluster[k].size(); ++l)
            {
                size_t idx = selected_cluster[k][l];
                std::vector<size_t> ltmp;
                ltmp = vi[idx].pointsInPointCloud(pc);

                lpoints.insert(lpoints.end(), ltmp.begin(), ltmp.end());
            }

            //change color of points to cluster color
            for (size_t i=0; i< lpoints.size(); ++i)
            {
                cv::Scalar c = cluster_colors.cv_color(k);
                int8_t r = c[2], g = c[1], b = c[0];
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                pc->points[lpoints[i]].rgb = *reinterpret_cast<float*>(&rgb);;
                pc2->points.push_back(pc->points[lpoints[i]]);

            }
        }


        pc2->is_dense = false;
        pc2->width = pc2->points.size();
        pc2->height = 1;
        sensor_msgs::PointCloud2 pcmsgout;
        pcl::toROSMsg(*pc2, pcmsgout);
        pub_pointcloud->publish(pcmsgout);

    }

    ros::Duration d = (ros::Time::now() - t);
    ROS_INFO("Comparisson took %f secs", d.toSec());

}

void compareCallbackUsingRegions(const ros::TimerEvent&)
{
// MODE 2: COMPARE OCTOMAP WITH REGIONS
// Compare a Octomap retrieved by a topic with Regions previously defined and check for inconsistencies.

    ros::Time t= ros::Time::now();

    ROS_INFO("Compare callback using regions triggered");

    // Checks if the OcTree Target was already received.
    if (octree_target == NULL)
    {
        ROS_INFO("OcTree Target Not Found");
        return;   
    }

    // Checks if the received target has changed, if not, exits the callback.
    if (flg_received_new_target==true)
    {
        flg_received_new_target = false;
    }
    else
    {
        return;
    }

    // Visualization Message Marker Array Initialization
    visualization_msgs::MarkerArray ma;
    visualization_msgs::MarkerArray ma_inconsistencies;
    visualization_msgs::MarkerArray ma_clusters;

    visualization_msgs::Marker marker_deleteall;
    marker_deleteall.header.stamp = ros::Time();
    marker_deleteall.header.frame_id = octree_frame_id ;
    marker_deleteall.ns = "target_inconsistent";
    marker_deleteall.action = 3;

    size_t id=0;
    size_t id_inconsistencies=0;
    size_t id_noneighbors=0;

    // Color initialization
    std_msgs::ColorRGBA color_occupied;
    color_occupied.r = 0; color_occupied.g = 0; color_occupied.b = 0.5; color_occupied.a = .8;

    std_msgs::ColorRGBA color_inconsistent;
    color_inconsistent.r = .5; color_inconsistent.g = 0; color_inconsistent.b = 0; color_inconsistent.a = .4;
    std_msgs::ColorRGBA color_inconsistent_missing;
    color_inconsistent_missing.r = .0; color_inconsistent_missing.g = 0.5; color_inconsistent_missing.b = 0; color_inconsistent_missing.a = .4;

    std_msgs::ColorRGBA color_target_volume;
    color_target_volume.r = .5; color_target_volume.g = 0.5; color_target_volume.b = 0; color_target_volume.a = 1;

    std_msgs::ColorRGBA color_noneighbors;
    color_noneighbors.r = .5; color_noneighbors.g = 0; color_noneighbors.b = 1; color_noneighbors.a = .8;


    // Vector of Inconsistencies Initialization
    std::vector<ClassBoundingBox> vi;
    std::vector<ClassBoundingBox> vi_missing;

    // Creates the target volume message array
    ma.markers.push_back(target_volume.getMarkerWithEdges("target_volume", octree_frame_id , color_target_volume, ++id));


    ROS_INFO_STREAM("Starting Iteration");

    // Iterates over each region
    for (size_t i=0; i < boxes.size(); ++i)
    {
        size_t num_occupied = 0;
        size_t num_neighbors = 0;

        // Iterates over target Octree
        for(OcTree::leaf_bbx_iterator it = octree_target->begin_leafs_bbx(boxes[i].getMinimumPoint(), boxes[i].getMaximumPoint(), depth), end=octree_target->end_leafs_bbx(); it!= end; ++it)
        {
            // Verifies if the node exists
            if (octree_target->search(it.getKey())) 
            {

                num_neighbors++;

                // Verifies if the node is free
                if (!octree_target->isNodeOccupied(*it)) 
                {
                    // Do nothing
                }
                
                else
                {
                    num_occupied++;
                }
            }
        }

            double occupation_ratio=0;
            
            // Occupation Ratio computation
            if (num_neighbors !=0)
            {
                occupation_ratio = (double)num_occupied/(double)num_neighbors;
            }

            // Checks for Inconsistencies of type EXCEEDING
            if (boxes[i].occupied == false && occupation_ratio >= exceeding_threshold_with_regions && num_neighbors !=0) 
            {
                // Add the inconsistency cell into a vector
                vi.push_back(boxes[i]);
            }

            // Checks for Inconsistencies of type MISSING
            if (boxes[i].occupied == true && occupation_ratio <= missing_threshold_with_regions && num_neighbors !=0) //If no occupied cell was found out of all iterated in the model's bbox, then an inconsistency is detected
                {
                    // Add the inconsistency cell into a vector
                    vi_missing.push_back(boxes[i]);
                }
    }


    //Cluster the EXCEEDING cells
    vector< vector<size_t> > cluster; 
    clusterBoundingBoxes(vi, cluster);
    ROS_INFO("There are %ld clusters", cluster.size());
    class_colormap cluster_colors("autumn", cluster.size(), 0.8);

    // Cluster the MISSING cells
    vector< vector<size_t> > cluster_missing; 
    clusterBoundingBoxes(vi_missing, cluster_missing);
    ROS_INFO("There are %ld clusters_missing", cluster_missing.size());
    class_colormap cluster_missing_colors("summer", cluster_missing.size(), 0.8);

    //Select only EXCEEDING clusters above a given volume threshold
    vector< vector<size_t> > selected_cluster; 
    filterClustersByVolume(vi, cluster, selected_cluster, volume_threshold);
    ROS_INFO("Selected %ld clusters using volume threshold", selected_cluster.size());

    //Select only MISSING clusters above a given volume threshold
    vector< vector<size_t> > selected_cluster_missing; 
    filterClustersByVolume(vi_missing, cluster_missing, selected_cluster_missing, volume_threshold);
    ROS_INFO("Selected %ld clusters_missing using volume threshold", selected_cluster_missing.size());

    //Draw filtered inconsistencies clusters in RVIZ
    class_colormap inconsistencies_colors(0.5,0,0,0.4);
    clustersToMarkerArray(vi, selected_cluster, ma_inconsistencies, id_inconsistencies, octree_frame_id, "inconsistencies", inconsistencies_colors);
    class_colormap inconsistencies_missing_colors(0,0.5,0,0.4);
    clustersToMarkerArray(vi_missing, selected_cluster_missing, ma_inconsistencies, id_inconsistencies, octree_frame_id, "inconsistencies", inconsistencies_missing_colors);

    // Draw all the clusters in RVIZ
    size_t id_clusters=0;
    clustersToMarkerArray(vi, selected_cluster, ma_clusters, id_clusters, octree_frame_id, "clusters", cluster_colors);
    clustersToMarkerArray(vi_missing, selected_cluster_missing, ma_clusters, id_clusters, octree_frame_id, "clusters", cluster_missing_colors);

    // Draw the Center of Mass Sphere and Volume Information
    visualization_msgs::MarkerArray ma_centerofmass;
    visualization_msgs::MarkerArray ma_volumeText;
    size_t id_ma_centerofmass = 0;
    centerOfMass(vi, selected_cluster, ma_centerofmass, ma_volumeText, id_ma_centerofmass, octree_frame_id);
    centerOfMass(vi_missing, selected_cluster_missing, ma_centerofmass, ma_volumeText, id_ma_centerofmass, octree_frame_id);

    // Publish the Marker Arrays
    marker_pub_inconsistencies->publish(ma_inconsistencies);
    marker_pub_clusters->publish(ma_clusters);
    marker_pub->publish(ma);
    marker_pub_center_of_mass->publish(ma_centerofmass);
    marker_pub_volume->publish(ma_volumeText);


    // Paint the Point Cloud (if available) with the Inconsistencies information.
    if (!flg_received_point_cloud)
    {
        ROS_ERROR_STREAM("No point_cloud2 has been received yet");
    }
    else
    {
        ROS_INFO("Processing point cloud ...");

        *pc = *pcin;
        
        pcl_ros::transformPointCloud(octree_frame_id, *pc, *pc, *listener);
        
        *pc2 = *pc;
        
        pc2->points.erase(pc2->points.begin(), pc2->points.end());

        for (size_t k = 0; k < selected_cluster.size(); ++k)
        {
            std::vector<size_t> lpoints;
            for (size_t l = 0; l < selected_cluster[k].size(); ++l)
            {
                size_t idx = selected_cluster[k][l];
                std::vector<size_t> ltmp;
                ltmp = vi[idx].pointsInPointCloud(pc);

                lpoints.insert(lpoints.end(), ltmp.begin(), ltmp.end());
            }

            // Change color of points to cluster color
            for (size_t i=0; i< lpoints.size(); ++i)
            {
                cv::Scalar c = cluster_colors.cv_color(k);
                int8_t r = c[2], g = c[1], b = c[0];
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                pc->points[lpoints[i]].rgb = *reinterpret_cast<float*>(&rgb);;
                pc2->points.push_back(pc->points[lpoints[i]]);

            }
        }

        pc2->is_dense = false;
        pc2->width = pc2->points.size();
        pc2->height = 1;
        sensor_msgs::PointCloud2 pcmsgout;
        pcl::toROSMsg(*pc2, pcmsgout);
        pub_pointcloud->publish(pcmsgout);

    }

    ros::Duration d = (ros::Time::now() - t);
    ROS_INFO("Comparisson took %f secs", d.toSec());
}

void callbackDynamicReconfigure(world_model_consistency_check::DepthConfigurationConfig &config, uint32_t level) 
{
    ROS_INFO("Reconfigure Request: Setting comparison depth to %d",  config.depth);
    ROS_INFO("Reconfigure Request: Setting volume threshold to %f",  config.volume_threshold);
    ROS_INFO("Reconfigure Request: Setting missing threshold to %f",  config.missing_threshold);
    ROS_INFO("Reconfigure Request: Setting exceeding threshold to %f",  config.exceeding_threshold);
    ROS_INFO("Reconfigure Request: Setting missing threshold with regions to %f", config.missing_threshold_with_regions);
    ROS_INFO("Reconfigure Request: Setting exceeding threshold with regions to %f", config.exceeding_threshold_with_regions);

    depth = (unsigned char) config.depth;
    volume_threshold = (double) config.volume_threshold;
    missing_threshold = (double) config.missing_threshold;
    exceeding_threshold = (double) config.exceeding_threshold;
    missing_threshold_with_regions = (double) config.missing_threshold_with_regions;
    exceeding_threshold_with_regions = (double) config.exceeding_threshold_with_regions;

}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "compare_octrees");
    nh = (boost::shared_ptr<ros::NodeHandle>) new ros::NodeHandle;

    // Use: _topic_model:=/topic_model  and  _topic_target:=/topic_target
    ros::param::get("~topic_model", topic_model);
    ros::param::get("~topic_target", topic_target);
    ros::param::get("~use_regions", use_regions);
    ros::param::get("~permanent_markers", permanent_markers);
    if (use_regions)
    {
        load_regions();
    }

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
    pc2 = (pcl::PointCloud<pcl::PointXYZRGB>::Ptr) new (pcl::PointCloud<pcl::PointXYZRGB>);
    pcin = (pcl::PointCloud<pcl::PointXYZRGB>::Ptr) new (pcl::PointCloud<pcl::PointXYZRGB>);

    ros::Subscriber sub_model;
    if (!use_regions)
    {
        sub_model = nh->subscribe(topic_model, 0, octomapCallbackModel);
    }

    ros::Subscriber sub_target = nh->subscribe(topic_target, 0, octomapCallbackTarget);
    ros::Subscriber sub_pointcloud = nh->subscribe(topic_point_cloud, 0, pointCloudCallback);

    ros::Timer timer;
    if (!use_regions)
    {
        timer = nh->createTimer(ros::Duration(0.3), compareCallback);
    }
    else
    {
        timer = nh->createTimer(ros::Duration(0.3), compareCallbackUsingRegions);
    }

    marker_pub = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub = nh->advertise<visualization_msgs::MarkerArray>("/target_volume", 10);

    marker_pub_center_of_mass = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub_center_of_mass = nh->advertise<visualization_msgs::MarkerArray>("/center_of_mass", 10);

    marker_pub_volume = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub_volume = nh->advertise<visualization_msgs::MarkerArray>("/volume", 10);

    marker_pub_inconsistencies = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub_inconsistencies = nh->advertise<visualization_msgs::MarkerArray>("/inconsistencies", 10);

    marker_pub_clusters = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub_clusters = nh->advertise<visualization_msgs::MarkerArray>("/clusters", 10);

    pub_pointcloud = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *pub_pointcloud = nh->advertise<sensor_msgs::PointCloud2>("/inconsistent_points", 0);




    ros::spin();
    return (0);
}
