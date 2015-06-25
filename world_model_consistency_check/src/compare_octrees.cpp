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

//My includes
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

    double cluster_volume = 0;

    for (size_t m = 0; m < cluster.size(); ++m)
    {

        double totalX = 0;
        double totalY = 0;
        double totalZ = 0;

        double cell_volume = 0;

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

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id ;
        marker.header.stamp = ros::Time();
        marker.ns = "centerOfMass";
        marker.id = id; //   ATENTION!!
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.5);

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

        cluster_volume += cell_volume;

        // Create the marker
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

        // double to string
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

        // marker_volume.text = std::string("Volume: ") + str_volume + "\n";
        marker_volume.text = std::string("X: ") + str_averageX + std::string(" Y: ") + str_averageY + std::string(" Z: ") + str_averageZ + "\n" + std::string("Volume: ") + str_volume;

        marker_volume.scale.z = 0.05; // Size of Text
        marker_volume.color.a = 1;
        marker_volume.color.r = 0;
        marker_volume.color.g = 0;
        marker_volume.color.b = 1;

        marker_volume.lifetime = ros::Duration(0.5);

        ma_volumeText.markers.push_back(marker_volume);

        id++;

    }
}


void clustersToMarkerArray(vector<ClassBoundingBox>& vi, vector< vector<size_t> >& cluster, visualization_msgs::MarkerArray& ma, size_t& id, string frame_id, string ns, class_colormap& cluster_colors)
{

    // Iterates once per cluster
    for (size_t k = 0; k < cluster.size(); ++k)
    {
        // Iterates once per point of the cluster
        for (size_t l = 0; l < cluster[k].size(); ++l)
        {
            size_t cluster_aux = cluster[k][l];
            ma.markers.push_back(vi[cluster_aux].getMarkerCubeVolume(ns, frame_id, cluster_colors.color(k), ++id));
        }
    }

}

void filterClustersByVolume(vector<ClassBoundingBox>& vi, vector< vector<size_t> >& cluster, vector< vector<size_t> >& selected_cluster, double volume_threshold)
{

    for (size_t k = 0; k < cluster.size(); ++k)
    {
        //Assume all cells have the same volume
        double cell_volume = ((ClassBoundingBox) vi[cluster[k][0]]).getVolume();
        double cluster_volume = cell_volume * cluster[k].size();

        if (cluster_volume > volume_threshold)
        {
            vector<size_t> tmp;
            // Iterates once per point of the cluster
            for (size_t l = 0; l < cluster[k].size(); ++l)
            {
                //size_t cluster_aux = cluster[k][l];
                tmp.push_back(cluster[k][l]);
            }
            selected_cluster.push_back(tmp);
        }
    }
}


void clusterBoundingBoxes(vector<ClassBoundingBox>& vi, vector< vector<size_t> >& cluster)
{

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

    while (queue.size() != 0)
    {
        //Select new seed
        size_t seed = queue[0]; 
        queue.erase(queue.begin() + 0); //remove first element

        // ROS_INFO("Selected seed point %ld, queue has size=%ld", seed, queue.size());

        //Create new cluster
        vector<size_t> tmp;
        cluster.push_back(tmp);

        //ROS_INFO("Created cluster %ld ", cluster.size());

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
                    //std::cout << 
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


        //ROS_INFO("Created cluster %ld with %ld points", cluster.size(), cluster[cluster.size()-1].size());
    }
}


bool are_neighbors(ClassBoundingBox b1, ClassBoundingBox b2)
{
    // double cx1 = b1.getCenter().x();
    // double cy1 = b1.getCenter().y();
    // double cz1 = b1.getCenter().z();

    // double cx2 = b2.getCenter().x();
    // double cy2 = b2.getCenter().y();
    // double cz2 = b2.getCenter().z();

    // double dist = sqrt((cx1-cx2)*(cx1-cx2) + (cy1-cy2)*(cy1-cy2) + (cz1-cz2)*(cz1-cz2));

    // double mx1 =  b1.getMinimumPoint().x();
    // double my1 =  b1.getMinimumPoint().y();
    // double mz1 =  b1.getMinimumPoint().z();
    // double s1 = sqrt( (cx1 - mx1) * (cx1 - mx1) + (cy1 - my1) * (cy1 - my1) + (cz1 - mz1) * (cz1 - mz1));

    // double mx2 =  b2.getMinimumPoint().x();
    // double my2 =  b2.getMinimumPoint().y();
    // double mz2 =  b2.getMinimumPoint().z();
    // double s2 = sqrt( (cx2 - mx2) * (cx2 - mx2) + (cy2 - my2) * (cy2 - my2) + (cz2 - mz2) * (cz2 - mz2));



    // //ROS_INFO("dx=%f dy=%f dz=%f", dx,dy,dz);
    // //ROS_INFO("dist=%f", dist);
    // //ROS_INFO("size_sum=%f", size_sum);
    // //if (dist <= size_sum*50000000000)
    // if (dist <= (s1 + s2))
    //     return true;
    // else
    //     return false;




    double cx1 = b1.getCenter().x();
    double cy1 = b1.getCenter().y();
    double cz1 = b1.getCenter().z();

    double cx2 = b2.getCenter().x();
    double cy2 = b2.getCenter().y();
    double cz2 = b2.getCenter().z();

    double mx1 =  b1.getMinimumPoint().x();
    double my1 =  b1.getMinimumPoint().y();
    double mz1 =  b1.getMinimumPoint().z();

    double mx2 =  b2.getMinimumPoint().x();
    double my2 =  b2.getMinimumPoint().y();
    double mz2 =  b2.getMinimumPoint().z();
    

    double dist_cx = sqrt((cx1-cx2)*(cx1-cx2));
    double dist_cy = sqrt((cy1-cy2)*(cy1-cy2));
    double dist_cz = sqrt((cz1-cz2)*(cx1-cz2));

    double dist_mx = abs(cx1 - mx1) + abs(cx2 - mx2);
    double dist_my = abs(cy1 - my1) + abs(cy2 - my2);
    double dist_mz = abs(cz1 - mz1) + abs(cz2 - mz2);

    if (dist_cx == dist_mx && dist_cz == dist_my && dist_cz == dist_mz)
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

unsigned char depth = 13;
double volume_threshold = 0.7;
double exceeding_threshold = 0.2;
double missing_threshold = 0.5;
double exceeding_threshold_with_regions = 0.1;
double missing_threshold_with_regions = 0.9;


//Declare a ClassBoundingBox which defines the target_volume
//ClassBoundingBox target_volume(-0.42, 0.42, -0.42, 0.42, 0.2, 3.0);
//ClassBoundingBox target_volume(0.6, 1.4, -.7, .7, 0.6, 2.0);
ClassBoundingBox target_volume(0.4, 2.0, -1.0, 1., 0.3, 2.2);
std::string octree_frame_id = "world";
bool flg_received_new_target = false;
bool flg_received_point_cloud = false;
AbstractOcTree* model_tree;
AbstractOcTree* target_tree;
std::vector<ClassBoundingBox> boxes;

/* _________________________________
   |                                 |
   |           Callbacks             |
   |_________________________________| */

void load_regions(void)
{
    std::vector<double> center_x;
    std::vector<double> center_y;
    std::vector<double> center_z;
    std::vector<double> size_x;
    std::vector<double> size_y;
    std::vector<double> size_z;
    std::vector<bool> is_occupied;
    std::string frame_id = "map";

    ros::param::get("/interactive_region_definition/center_x", center_x);
    ros::param::get("/interactive_region_definition/center_y", center_y);
    ros::param::get("/interactive_region_definition/center_z", center_z);
    ros::param::get("/interactive_region_definition/size_x", size_x);
    ros::param::get("/interactive_region_definition/size_y", size_y);
    ros::param::get("/interactive_region_definition/size_z", size_z);
    ros::param::get("/interactive_region_definition/is_occupied", is_occupied);
    ros::param::get("/interactive_region_definition/frame_id", frame_id);

    ROS_INFO("There are %ld loaded boxes", center_x.size());

    boxes.erase(boxes.begin(), boxes.end());
    ROS_INFO("There are %ld boxes in memory", boxes.size());
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
    //ROS_INFO("Received new octree model on topic %s with id: %s, frame_id: %s\n", topic_model.c_str(), msg->id.c_str(), msg->header.frame_id.c_str());
    model_tree = msgToMap(*msg);
    octree_model = dynamic_cast<OcTree*>(model_tree);
    //free(model_tree);
}

void octomapCallbackTarget(const octomap_msgs::Octomap::ConstPtr& msg)
{
    //ROS_INFO("Received new octree target on topic %s with id: %s, frame_id: %s\n", topic_target.c_str(), msg->id.c_str(), msg->header.frame_id.c_str());
    target_tree = msgToMap(*msg);
    octree_target = dynamic_cast<OcTree*>(target_tree);
    //free(target_tree);

    if (msg->header.frame_id != "")
        octree_frame_id = msg->header.frame_id;

    flg_received_new_target = true;
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, *pcin);
    //ROS_INFO("Received point cloud with frame_id %s", pcin )
    flg_received_point_cloud = true;
}

void compareCallbackUsingRegions(const ros::TimerEvent&)
{
    ros::Time t= ros::Time::now();

    ROS_INFO("Compare callback using regions triggered");

    // Checks if the OcTree Target was already received
    if (octree_target == NULL)
    {
        ROS_INFO("OcTree Target Not Found");
        return;   
    }

    if (flg_received_new_target==true)
    {
        flg_received_new_target = false;
    }
    else
    {
        return;
    }

    // Visualization Message Marker Array
    visualization_msgs::MarkerArray ma;
    visualization_msgs::MarkerArray ma_inconsistencies;
    visualization_msgs::MarkerArray ma_clusters;

    visualization_msgs::Marker marker_deleteall;
    marker_deleteall.header.stamp = ros::Time();
    marker_deleteall.header.frame_id = octree_frame_id ;
    marker_deleteall.ns = "target_inconsistent";
    marker_deleteall.action = 3;

    unsigned int id=0;
    unsigned int id_inconsistencies=0;
    unsigned int id_noneighbors=0;

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


    // Vector of Inconsistencies initialization
    std::vector<ClassBoundingBox> vi;
    std::vector<ClassBoundingBox> vi_missing;

    // Creates the target volume message array
    ma.markers.push_back(target_volume.getMarkerWithEdges("target_volume", octree_frame_id , color_target_volume, ++id));


    ROS_INFO_STREAM("Starting Iteration");


    for (size_t i=0; i < boxes.size(); ++i)
    {
        size_t num_occupied = 0;
        size_t num_neighbors = 0;

        // -------------------------------------------------------------
        // ----------- Iterate over target octree ----------------------
        // -------------------------------------------------------------
        for(OcTree::leaf_bbx_iterator it = octree_target->begin_leafs_bbx(boxes[i].getMinimumPoint(), boxes[i].getMaximumPoint(), depth), end=octree_target->end_leafs_bbx(); it!= end; ++it)
        {
            if (octree_target->search(it.getKey())) // Verifies if the node exists
            {
                //if (octree_target->isNodeOccupied(*it)) 
                //{
                //ClassBoundingBox target_cell(it.getX(), it.getY(), it.getZ(), it.getSize());

                //ma.markers.push_back(target_cell.getMarkerWithEdges("target_occupied", octree_frame_id , color_occupied, ++id));

                num_neighbors++;

                if (!octree_target->isNodeOccupied(*it)) // Verifies if the node is free
                {
                    //Do something here - Draw node, etc
                }
                else
                {
                    num_occupied++;
                    //flg_found_at_least_one_occupied = true;
                }

            }
        }

            double occupation_ratio=0;
            if (num_neighbors !=0)
            {
                occupation_ratio = (double)num_occupied/(double)num_neighbors;
            }


            if (boxes[i].occupied == false && occupation_ratio >= exceeding_threshold_with_regions && num_neighbors !=0) //If no occupied cell was found out of all iterated in the model's bbox, then an inconsistency is detected
            {
                //Inconsistencies of type exceeding 
                // Add the inconsistency cell into a vector
                vi.push_back(boxes[i]);

                ma_inconsistencies.markers.push_back(boxes[i].getMarkerCubeVolume("target_inconsistent", octree_frame_id, color_inconsistent, ++id_inconsistencies));
            }

            if (boxes[i].occupied == true && occupation_ratio <= missing_threshold_with_regions && num_neighbors !=0) //If no occupied cell was found out of all iterated in the model's bbox, then an inconsistency is detected
                {
                    //Inconsistencies of type exceeding 
                    // Add the inconsistency cell into a vector
                    vi_missing.push_back(boxes[i]);

                    ma_inconsistencies.markers.push_back(boxes[i].getMarkerCubeVolume("target_inconsistent", octree_frame_id, color_inconsistent_missing, ++id_inconsistencies));
                }
    }



    //Cluster the exceeding bounding boxes
    vector< vector<size_t> > cluster; 
    clusterBoundingBoxes(vi, cluster);
    ROS_INFO("There are %ld clusters", cluster.size());
    class_colormap cluster_colors("autumn", cluster.size(), 0.8);

    vector< vector<size_t> > cluster_missing; 
    clusterBoundingBoxes(vi_missing, cluster_missing);
    ROS_INFO("There are %ld clusters_missing", cluster_missing.size());
    class_colormap cluster_missing_colors("summer", cluster_missing.size(), 0.8);

    //Select only clusters above a given volume threshold
    vector< vector<size_t> > selected_cluster; 
    filterClustersByVolume(vi, cluster, selected_cluster, volume_threshold);
    ROS_INFO("Selected %ld clusters using volume threshold", selected_cluster.size());

    vector< vector<size_t> > selected_cluster_missing; 
    filterClustersByVolume(vi_missing, cluster_missing, selected_cluster_missing, volume_threshold);
    ROS_INFO("Selected %ld clusters_missing using volume threshold", selected_cluster_missing.size());

    //Draw selected clusters in RVIZ
    size_t id_clusters=0;
    clustersToMarkerArray(vi, selected_cluster, ma_clusters, id_clusters, octree_frame_id, "clusters", cluster_colors);
    clustersToMarkerArray(vi_missing, selected_cluster_missing, ma_clusters, id_clusters, octree_frame_id, "clusters", cluster_missing_colors);

    // Draw the Center of Mass Sphere and Volume Information
    visualization_msgs::MarkerArray ma_centerofmass;
    visualization_msgs::MarkerArray ma_volumeText;
    size_t id_ma_centerofmass = 0;
    centerOfMass(vi, selected_cluster, ma_centerofmass, ma_volumeText, id_ma_centerofmass, octree_frame_id);
    centerOfMass(vi_missing, selected_cluster_missing, ma_centerofmass, ma_volumeText, id_ma_centerofmass, octree_frame_id);




    // }


    // /* ______________________________________
    //    |                                      |
    //    |    Exceeding Clusters                |
    //    |________________________________      | */

    // // ----------------------------------------------------
    // // ----------- Print Volume of Clusters ---------------
    // // ----------------------------------------------------

    // // Visualization Message Marker Array for the center of mass
    // visualization_msgs::MarkerArray ma_volumeText;
    // int id_ma_volume = 0;

    // for (size_t m = 0; m < selected_cluster.size(); ++m)
    // {

    //     double totalX = 0;
    //     double totalY = 0;
    //     double totalZ = 0;

    //     for (size_t n = 0; n < selected_cluster[m].size(); ++n)
    //     {

    //         size_t cluster_aux = selected_cluster[m][n];

    //         // double total_volume += vi[cluster_aux].getVolume();
    //         // double totalX += vi[cluster_aux].getCenter().x() * vi[cluster_aux].getVolume();

    //         // Calculate the sum of X
    //         totalX += vi[cluster_aux].getCenter().x();

    //         // Calculate the sum of Y
    //         totalY += vi[cluster_aux].getCenter().y();

    //         // Calculate the sum of Z
    //         totalZ += vi[cluster_aux].getCenter().z();
    //     }

    //     // Calculate the average of X
    //     double averageX = 0;
    //     averageX = totalX / selected_cluster[m].size();

    //     // Calculate the average of Y
    //     double averageY = 0;
    //     averageY = totalY / selected_cluster[m].size();

    //     // Calculate the average of Z
    //     double averageZ = 0;
    //     averageZ = totalZ / selected_cluster[m].size();

    //     // Compute the volume
    //     //Assume all cells have the same volume
    //     double cell_volume = vi[selected_cluster[m][0]].getVolume();
    //     double cluster_volume = cell_volume * selected_cluster[m].size();

    //     // Create the marker
    //     visualization_msgs::Marker marker_volume;
    //     marker_volume.header.frame_id = octree_frame_id ;
    //     marker_volume.header.stamp = ros::Time();
    //     marker_volume.ns = "volume";
    //     marker_volume.id = id_ma_volume; //   ATENTION!!
    //     marker_volume.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    //     marker_volume.action = visualization_msgs::Marker::ADD;

    //     marker_volume.pose.position.x = averageX;
    //     marker_volume.pose.position.y = averageY;
    //     marker_volume.pose.position.z = averageZ;

    //     // double to string
    //     std::ostringstream os;
    //     char ss[1024];

    //     sprintf(ss, "%0.6f", cluster_volume);
    //     std::string str_volume = ss;

    //     sprintf(ss, "%0.3f", averageX);
    //     std::string str_averageX = ss;

    //     sprintf(ss, "%0.3f", averageY);
    //     std::string str_averageY = ss;

    //     sprintf(ss, "%0.3f", averageZ);
    //     std::string str_averageZ = ss;

    //     // marker_volume.text = std::string("Volume: ") + str_volume + "\n";
    //     marker_volume.text = std::string("X: ") + str_averageX + std::string(" Y: ") + str_averageY + std::string(" Z: ") + str_averageZ + "\n" + std::string("Volume: ") + str_volume;

    //     marker_volume.scale.z = 0.05; // Size of Text
    //     marker_volume.color.a = 1;
    //     marker_volume.color.r = 1;
    //     marker_volume.color.g = 1;
    //     marker_volume.color.b = 1;

    //     marker_volume.lifetime = ros::Duration(0.5);

    //     ma_volumeText.markers.push_back(marker_volume);

    //     id_ma_volume++;


    // }





    //Delete
    //visualization_msgs::MarkerArray ma_deleteall;
    //visualization_msgs::Marker marker;
    //marker.header.stamp = ros::Time();
    //marker.header.frame_id = octree_frame_id ;
    //marker.ns = "clusters";
    ////marker.action = visualization_msgs::Marker::DELETEALL;
    //marker.action = 3;
    //ma_deleteall.markers.push_back(marker);
    //marker_pub_clusters->publish(ma_deleteall);

    //ma_deleteall.markers[0].ns = "target_inconsistent";
    //marker_pub_inconsistencies->publish(ma_deleteall);
    //marker_pub_inconsistencies->publish(ma_deleteall);

    //ros::Duration(0.05).sleep();

    //marker_deleteall.ns = "target_inconsistent";
    //ma_inconsistencies.markers.insert(ma_inconsistencies.markers.begin(), 0, marker_deleteall);
    marker_pub_inconsistencies->publish(ma_inconsistencies);
    marker_pub_clusters->publish(ma_clusters);

    marker_pub->publish(ma);

    marker_pub_center_of_mass->publish(ma_centerofmass);

    marker_pub_volume->publish(ma_volumeText);

    //publish colored point cloud
    //double time_to_wait_for_point_cloud = 0.1;
    //ros::Time t_point_cloud = ros::Time::now();
    //ROS_INFO_STREAM("Waiting for a point_cloud2 on topic " << "/camera/depth_registered/points");
    //sensor_msgs::PointCloud2::ConstPtr pcmsg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", *nh, ros::Duration(time_to_wait_for_point_cloud));
    //ros::spinOnce();
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

    if (flg_received_new_target==true)
    {
        flg_received_new_target = false;
    }
    else
    {
        return;
    }
    // Visualization Message Marker Array
    visualization_msgs::MarkerArray ma;
    visualization_msgs::MarkerArray ma_inconsistencies;
    visualization_msgs::MarkerArray ma_clusters;

    visualization_msgs::Marker marker_deleteall;
    marker_deleteall.header.stamp = ros::Time();
    marker_deleteall.header.frame_id = octree_frame_id ;
    marker_deleteall.ns = "target_inconsistent";
    marker_deleteall.action = 3;

    unsigned int id=0;
    unsigned int id_inconsistencies=0;
    unsigned int id_noneighbors=0;
    unsigned int id_clusters=0;

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

    // -------------------------------------------------------------
    // ----------- Iterate over target octree ----------------------
    // -------------------------------------------------------------
    for(OcTree::leaf_bbx_iterator it = octree_target->begin_leafs_bbx(target_volume.getMinimumPoint(), target_volume.getMaximumPoint(), depth), end=octree_target->end_leafs_bbx(); it!= end; ++it)
    {
        if (octree_target->search(it.getKey())) // Verifies if the node exists
        {
            if (octree_target->isNodeOccupied(*it)) 
            {
                ClassBoundingBox target_cell(it.getX(), it.getY(), it.getZ(), it.getSize());
                //bool flg_found_at_least_one_occupied = false;
                //bool flg_found_neighbors = false;
                //int count =0;
                size_t num_occupied = 0;
                size_t num_neighbors = 0;

                ma.markers.push_back(target_cell.getMarkerWithEdges("target_occupied", octree_frame_id , color_occupied, ++id));

                // -------------------------------------------------------------
                // ----------- Iterate over model octree ----------------------
                // -------------------------------------------------------------
                for(OcTree::leaf_bbx_iterator it_model = octree_model->begin_leafs_bbx(target_cell.getMinimumPoint(),target_cell.getMaximumPoint(), depth), end=octree_model->end_leafs_bbx(); it_model!= end; ++it_model)
                {
                    if (octree_model->search(it_model.getKey())) // Verifies if the nodes exists
                    {
                        num_neighbors++;
                        //flg_found_neighbors = true;

                        if (!octree_model->isNodeOccupied(*it_model)) // Verifies if the node is free
                        {
                            //Do something here - Draw node, etc
                        }
                        else
                        {
                            num_occupied++;
                            //flg_found_at_least_one_occupied = true;
                        }
                    }
                }

                double occupation_ratio=0;
                if (num_neighbors !=0)
                {
                    occupation_ratio = (double)num_occupied/(double)num_neighbors;
                }


                if (occupation_ratio <= exceeding_threshold && num_neighbors !=0) //If no occupied cell was found out of all iterated in the model's bbox, then an inconsistency is detected
                {
                    //Inconsistencies of type exceeding 
                    // Add the inconsistency cell into a vector
                    vi.push_back(target_cell);

                    ma_inconsistencies.markers.push_back(target_cell.getMarkerCubeVolume("target_inconsistent", octree_frame_id, color_inconsistent, ++id_inconsistencies));
                }
            }
        }
    }



    // -------------------------------------------------------------
    // ----------- Iterate over model octree ----------------------
    // -------------------------------------------------------------
    for(OcTree::leaf_bbx_iterator it = octree_model->begin_leafs_bbx(target_volume.getMinimumPoint(), target_volume.getMaximumPoint(), depth), end=octree_model->end_leafs_bbx(); it!= end; ++it)
    {
        if (octree_model->search(it.getKey())) // Verifies if the node exists
        {
            if (octree_model->isNodeOccupied(*it))
            {
                ClassBoundingBox model_cell(it.getX(), it.getY(), it.getZ(), it.getSize());
                //bool flg_found_at_least_one_occupied = false;
                //bool flg_found_neighbors = false;
                //int count =0;
                size_t num_occupied = 0;
                size_t num_neighbors = 0;

                ma.markers.push_back(model_cell.getMarkerWithEdges("model_occupied", octree_frame_id , color_occupied, ++id));

                // -------------------------------------------------------------
                // ----------- Iterate over model octree ----------------------
                // -------------------------------------------------------------
                for(OcTree::leaf_bbx_iterator it_target = octree_target->begin_leafs_bbx(model_cell.getMinimumPoint(),model_cell.getMaximumPoint(), depth), end=octree_target->end_leafs_bbx(); it_target!= end; ++it_target)
                {
                    if (octree_target->search(it_target.getKey())) // Verifies if the nodes exists
                    {
                        num_neighbors++;
                        //flg_found_neighbors = true;

                        if (!octree_target->isNodeOccupied(*it_target)) // Verifies if the node is free
                        {
                            //Do something here - Draw node, etc
                        }
                        else
                        {
                            num_occupied++;
                            //flg_found_at_least_one_occupied = true;
                        }
                    }
                }

                double occupation_ratio=0;
                if (num_neighbors !=0)
                {
                    occupation_ratio = (double)num_occupied/(double)num_neighbors;
                }


                //if (octree_target->isNodeOccupied(*it) && flg_found_at_least_one_occupied == false && flg_found_neighbors == true) //If no occupied cell was found out of all iterated in the model's bbox, then an inconsistency is detected
                if (occupation_ratio <= missing_threshold && num_neighbors !=0) //If no occupied cell was found out of all iterated in the model's bbox, then an inconsistency is detected
                {
                    //Inconsistencies of type exceeding 
                    // Add the inconsistency cell into a vector
                    vi_missing.push_back(model_cell);

                    ma_inconsistencies.markers.push_back(model_cell.getMarkerCubeVolume("target_inconsistent", octree_frame_id, color_inconsistent_missing, ++id_inconsistencies));
                }
            }
        }
    }



    /* _________________________________
       |                                 |
       |     for the exceeding clusters  |
       |_________________________________| */

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

        // ROS_INFO("Selected seed point %ld, queue has size=%ld", seed, queue.size());

        //Create new cluster
        vector<size_t> tmp;
        cluster_missing.push_back(tmp);

        ROS_INFO("Created cluster_missing %ld ", cluster_missing.size());

        //Expand seed
        vector <size_t> flood;
        flood.push_back(seed);


        while (flood.size() != 0)
        {

            //ROS_INFO("Expanding first elem of flood (size %ld) idx = %ld", flood.size(), flood[0]);
            //expand flood[j]
            size_t idx_b1 = flood[0];


            //ROS_INFO("Checking of queue size %ld", queue.size());
            for (size_t j=0; j < queue_missing.size(); ++j) 
            {
                size_t idx_b2 = queue_missing[j]; 

                //ROS_INFO("Checking idx_b1 %ld idx_b2 %ld", idx_b1, idx_b2);


                //char name[50];
                //cout << "press a key to continue";
                //cin >> name;

                if (are_neighbors(vi_missing[idx_b1], vi_missing[idx_b2]))
                {
                    //ROS_INFO("Found neighbor idx %ld", idx_b2);
                    flood.push_back(idx_b2);
                    queue_missing.erase(queue_missing.begin() + j);
                    //TODO should be b2 or b1?

                }
                else
                {
                    //nothing to do 
                }
            }



            //add first elem of floodto cluster
            cluster_missing.at(cluster_missing.size()-1).push_back(flood[0]); //add seed point to cluster

            //remove first elem of  flood
            flood.erase(flood.begin() + 0);

        }


        ROS_INFO("Created cluster_missing %ld with %ld points", cluster_missing.size(), cluster_missing[cluster_missing.size()-1].size());



    }

    //Information about clusters

    ROS_INFO("There are %ld clusters", cluster.size());
    class_colormap cluster_colors("autumn", cluster.size(), 0.8);

    ROS_INFO("There are %ld clusters_missing", cluster_missing.size());
    class_colormap cluster_missing_colors("winter", cluster_missing.size(), 0.8, true);

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





    ROS_INFO("Inconsistencies vector has %ld cells", vi.size());


    /* ______________________________________
       |                                      |
       |    Exceeding Clusters                |
       |________________________________      | */
    //Filter clusters using volume threshold |

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


    /* ______________________________________
       | 
       |    Missing Clusters                  |
       |________________________________      | */
    //Filter clusters using volume threshold |
    vector< vector<size_t> > selected_cluster_missing; 

    for (size_t k = 0; k < cluster_missing.size(); ++k)
    {
        //Assume all cells have the same volume
        double cell_volume = vi_missing[cluster_missing[k][0]].getVolume();
        double cluster_volume = cell_volume * cluster_missing[k].size();

        if (cluster_volume > volume_threshold)
        {
            vector<size_t> tmp;
            // Iterates once per point of the cluster
            for (size_t l = 0; l < cluster_missing[k].size(); ++l)
            {
                size_t cluster_aux = cluster_missing[k][l];
                tmp.push_back(cluster_missing[k][l]);
            }
            selected_cluster_missing.push_back(tmp);
        }
    }
    ROS_INFO("Selected %ld clusters_missing using volume threshold", selected_cluster_missing.size());



    // ----------------------------------------------------
    // --------- Draws clusters on visualizer -------------

    // Iterates once per cluster
    for (size_t k = 0; k < selected_cluster_missing.size(); ++k)
    {
        // Iterates once per point of the cluster_missing
        for (size_t l = 0; l < selected_cluster_missing[k].size(); ++l)
        {
            size_t cluster_aux = selected_cluster_missing[k][l];
            ma_clusters.markers.push_back(vi_missing[cluster_aux].getMarkerCubeVolume("clusters", octree_frame_id, cluster_missing_colors.color(k), ++id_clusters));
        }
    }





    /* ______________________________________
       |                                      |
       |    Exceeding Clusters                |
       |________________________________      | */

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

        ROS_INFO("Averages for Cluster Exceeding[%ld]: X: %f, Y: %f, Z: %f", m, averageX, averageY, averageZ);

        visualization_msgs::Marker marker;
        marker.header.frame_id = octree_frame_id ;
        marker.header.stamp = ros::Time();
        marker.ns = "centerOfMass";
        marker.id = id_ma_centerofmass; //   ATENTION!!
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.5);

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


    /* ______________________________________
       |                                      |
       |    Exceeding Clusters                |
       |________________________________      | */

    // ----------------------------------------------------
    // ----------- Print Volume of Clusters ---------------
    // ----------------------------------------------------

    // Visualization Message Marker Array for the center of mass
    visualization_msgs::MarkerArray ma_volumeText;
    int id_ma_volume = 0;

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

        // Compute the volume
        //Assume all cells have the same volume
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

        // double to string
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

        // marker_volume.text = std::string("Volume: ") + str_volume + "\n";
        marker_volume.text = std::string("X: ") + str_averageX + std::string(" Y: ") + str_averageY + std::string(" Z: ") + str_averageZ + "\n" + std::string("Volume: ") + str_volume;

        marker_volume.scale.z = 0.05; // Size of Text
        marker_volume.color.a = 1;
        marker_volume.color.r = 1;
        marker_volume.color.g = 1;
        marker_volume.color.b = 1;

        marker_volume.lifetime = ros::Duration(0.5);

        ma_volumeText.markers.push_back(marker_volume);

        id_ma_volume++;


    }



    /* ______________________________________
       |                                      |
       |    Missing Clusters                  |
       |________________________________      | */

    // ----------------------------------------------------
    // ----------- Center of Mass of Clusters -------------
    // ----------------------------------------------------

    // // Visualization Message Marker Array for the center of mass
    // visualization_msgs::MarkerArray ma_centerofmass;
    // int id_ma_centerofmass = 0;


    for (size_t m = 0; m < selected_cluster_missing.size(); ++m)
    {

        double totalX = 0;
        double totalY = 0;
        double totalZ = 0;

        for (size_t n = 0; n < selected_cluster_missing[m].size(); ++n)
        {

            size_t cluster_aux = selected_cluster_missing[m][n];

            // double total_volume += vi[cluster_aux].getVolume();
            // double totalX += vi[cluster_aux].getCenter().x() * vi[cluster_aux].getVolume();

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
        marker.lifetime = ros::Duration(0.5);

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
    //visualization_msgs::MarkerArray ma_deleteall;
    //visualization_msgs::Marker marker;
    //marker.header.stamp = ros::Time();
    //marker.header.frame_id = octree_frame_id ;
    //marker.ns = "clusters";
    ////marker.action = visualization_msgs::Marker::DELETEALL;
    //marker.action = 3;
    //ma_deleteall.markers.push_back(marker);
    //marker_pub_clusters->publish(ma_deleteall);

    //ma_deleteall.markers[0].ns = "target_inconsistent";
    //marker_pub_inconsistencies->publish(ma_deleteall);
    //marker_pub_inconsistencies->publish(ma_deleteall);

    //ros::Duration(0.05).sleep();

    //marker_deleteall.ns = "target_inconsistent";
    //ma_inconsistencies.markers.insert(ma_inconsistencies.markers.begin(), 0, marker_deleteall);
    marker_pub_inconsistencies->publish(ma_inconsistencies);
    marker_pub_clusters->publish(ma_clusters);

    marker_pub->publish(ma);

    marker_pub_center_of_mass->publish(ma_centerofmass);

    marker_pub_volume->publish(ma_volumeText);

    //publish colored point cloud
    //double time_to_wait_for_point_cloud = 0.1;
    //ros::Time t_point_cloud = ros::Time::now();
    //ROS_INFO_STREAM("Waiting for a point_cloud2 on topic " << "/camera/depth_registered/points");
    //sensor_msgs::PointCloud2::ConstPtr pcmsg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", *nh, ros::Duration(time_to_wait_for_point_cloud));
    //ros::spinOnce();
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
