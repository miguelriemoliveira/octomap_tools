#ifndef _BOUNDING_BOX_H_
#define _BOUNDING_BOX_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>

#include <colormap/colormap.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>

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


        visualization_msgs::Marker getMarkerWithEdges(std::string ns, std::string frame_id, std_msgs::ColorRGBA color, int id, bool permanent_markers=false)
        {
            visualization_msgs::Marker m = createMarker(ns, frame_id, color, id);
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.scale.x = 0.02;
            if (permanent_markers==false)
                m.lifetime = ros::Duration(0.5);
            else
                m.lifetime = ros::Duration(0);

            getEdgesToDraw(m.points);
            return m;
        }

        visualization_msgs::Marker getMarkerCubeVolume(std::string ns, std::string frame_id, std_msgs::ColorRGBA color, int id, bool permanent_markers=false)
        {

            visualization_msgs::Marker m = createMarker(ns, frame_id, color, id);
            m.type = visualization_msgs::Marker::CUBE;
            //double size = _max_x - _min_x;
            //m.scale.x = m.scale.y = m.scale.z = size;
            //NOTE Check if it works
            m.scale.x = _max_x - _min_x; 
            m.scale.y = _max_y - _min_y;
            m.scale.z = _max_z - _min_z;
            m.pose.position.x = (_max_x + _min_x)/2;
            m.pose.position.y = (_max_y + _min_y)/2;
            m.pose.position.z = (_max_z + _min_z)/2;
            if (permanent_markers==false)
                m.lifetime = ros::Duration(0.5);
            else
                m.lifetime = ros::Duration(0);
            return m;
        }

        double getSize(void) {return _max_x - _min_x;};

        void setSize(double x, double y, double z)
        {
            point3d p =  getCenter(); 
            _min_x = p.x() - x/2;
            _max_x = p.x() + x/2;

            _min_y = p.y() - y/2;
            _max_y = p.y() + y/2;

            _min_z = p.z() - z/2;
            _max_z = p.z() + z/2;
        };


        double getSizeX(void) {return _max_x - _min_x;};
        double getSizeY(void) {return _max_y - _min_y;};
        double getSizeZ(void) {return _max_z - _min_z;};
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

        void setBoundingBox(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
        {
            _min_x = min_x; _max_x = max_x;
            _min_y = min_y; _max_y = max_y;
            _min_z = min_z; _max_z = max_z;
        }

    public: 
        bool occupied; //

    protected:

        double _min_x;
        double _max_x;
        double _min_y;
        double _max_y;
        double _min_z;
        double _max_z;

};

#endif

