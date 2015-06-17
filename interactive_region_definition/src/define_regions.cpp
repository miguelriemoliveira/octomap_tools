#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


#include <math.h>

#include <world_model_consistency_check/bounding_box.h>

#include <ros/package.h> //This include is sufficient to use ros::package::getPath
//#include <rospack/rospack.h>


using namespace visualization_msgs;
//using namespace ros;
//using namespace ros::package;



boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
boost::shared_ptr<ros::NodeHandle> nh;
boost::shared_ptr<ros::Publisher> marker_pub;
boost::shared_ptr<ros::Publisher> marker_pub_boxes;

ClassBoundingBox target_volume(0, 2, 0, 2, 0, 2);
std::vector<ClassBoundingBox> boxes;

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

    ros::param::get("~center_x", center_x);
    ros::param::get("~center_y", center_y);
    ros::param::get("~center_z", center_z);
    ros::param::get("~size_x", size_x);
    ros::param::get("~size_y", size_y);
    ros::param::get("~size_z", size_z);
    ros::param::get("~is_occupied", is_occupied);
    ros::param::get("~frame_id", frame_id);

    ROS_ERROR("There are %ld loaded boxes", center_x.size());

    boxes.erase(boxes.begin(), boxes.end());
    ROS_ERROR("There are %ld boxes in memory", boxes.size());
    for (size_t i=0; i< center_x.size(); ++i)
    {
        ClassBoundingBox b(center_x[i] - size_x[i]/2, center_x[i] + size_x[i]/2, center_y[i] - size_y[i]/2, center_y[i] + size_y[i]/2, center_z[i] - size_z[i]/2, center_z[i] + size_z[i]/2);
        b.occupied = is_occupied[i];
        boxes.push_back(b);
    }

    ROS_ERROR("Now there are %ld boxes in memory", boxes.size());


}


Marker makeBox( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::SPHERE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;

    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}

void drawingCallback(const ros::TimerEvent&)
{
    visualization_msgs::MarkerArray ma;
    std_msgs::ColorRGBA color;
    color.r = .6; color.g = 0.0; color.b = 0; color.a = 1;
    std_msgs::ColorRGBA color_free;
    color_free.r = 0.0; color_free.g = 0.6; color_free.b = 0; color_free.a = 1;

    size_t id = 0;

    for (size_t i = 0; i< boxes.size(); ++i)
    {
        if (boxes[i].occupied == true)
            ma.markers.push_back(boxes[i].getMarkerWithEdges("boxes", "/map" , color, ++id));
        else
            ma.markers.push_back(boxes[i].getMarkerWithEdges("boxes", "/map" , color_free, ++id));

    }
    marker_pub_boxes->publish(ma);

}


void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' " << " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if( feedback->mouse_point_valid )
    {
        mouse_point_ss << " at " << feedback->mouse_point.x
            << ", " << feedback->mouse_point.y
            << ", " << feedback->mouse_point.z
            << " in frame " << feedback->header.frame_id;
    }

    geometry_msgs::Pose pose = feedback->pose;

    std_msgs::ColorRGBA color;
    visualization_msgs::MarkerArray ma;
    int id = 0;


    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
            if (feedback->menu_entry_id==2)
            {
                boxes.push_back(target_volume);
                boxes[boxes.size()-1].occupied = false;
            }
            else if (feedback->menu_entry_id==3)
            {
                boxes.push_back(target_volume);
                boxes[boxes.size()-1].occupied = true;
            }
            else if (feedback->menu_entry_id==5) //save to file
            {
                ros::NodeHandle nh("~");

                std::vector<double> center_x;
                std::vector<double> center_y;
                std::vector<double> center_z;
                std::vector<double> size_x;
                std::vector<double> size_y;
                std::vector<double> size_z;
                std::vector<bool> is_occupied;
                std::string frame_id = "map";


                for (size_t i=0; i < boxes.size(); ++i)
                {
                    point3d p = boxes[i].getCenter();
                    center_x.push_back(p.x());
                    center_y.push_back(p.y());
                    center_z.push_back(p.z());

                    size_x.push_back(boxes[i].getSizeX());
                    size_y.push_back(boxes[i].getSizeY());
                    size_z.push_back(boxes[i].getSizeZ());

                    is_occupied.push_back(boxes[i].occupied);
                }

                nh.setParam("center_x", center_x);
                nh.setParam("center_y", center_y);
                nh.setParam("center_z", center_z);

                nh.setParam("size_x", size_x);
                nh.setParam("size_y", size_y);
                nh.setParam("size_z", size_z);

                nh.setParam("is_occupied", is_occupied);
                nh.setParam("frame_id", frame_id);

                //Problem linking? check http://answers.ros.org/question/196935/roslib-reference-error/
                string path = ros::package::getPath("interactive_region_definition");

                string cmd = "rosparam dump " + path + "/params/default.yaml " + "/interactive_region_definition";
                system(cmd.c_str());


            }
            else if (feedback->menu_entry_id==6) //load from file
            {

                //Problem linking? check http://answers.ros.org/question/196935/roslib-reference-error/
                string path = ros::package::getPath("interactive_region_definition");
                string cmd = "rosparam load " + path + "/params/default.yaml " + "/interactive_region_definition";
                system(cmd.c_str());

                ros::NodeHandle nh("~");

                std::vector<double> center_x;
                std::vector<double> center_y;
                std::vector<double> center_z;
                std::vector<double> size_x;
                std::vector<double> size_y;
                std::vector<double> size_z;
                std::vector<bool> is_occupied;
                std::string frame_id = "map";

                ros::param::get("~center_x", center_x);
                ros::param::get("~center_y", center_y);
                ros::param::get("~center_z", center_z);

                ros::param::get("~size_x", size_x);
                ros::param::get("~size_y", size_y);
                ros::param::get("~size_z", size_z);

                ros::param::get("~is_occupied", is_occupied);
                ros::param::get("~frame_id", frame_id);

                ROS_ERROR("There are %ld loaded boxes", center_x.size());

                ROS_ERROR("There are %ld boxes in memory", boxes.size());
                //boxes.erase(boxes.begin(), boxes.end());
                for (size_t i=0; i< center_x.size(); ++i)
                {
                    ROS_INFO("center_x[%ld]= %f", i, center_x[i]);
                    ClassBoundingBox b(center_x[i] - size_x[i]/2, center_x[i] + size_x[i]/2, center_y[i] - size_y[i]/2, center_y[i] + size_y[i]/2, center_z[i] - size_z[i]/2, center_z[i] + size_z[i]/2);
                    b.occupied = is_occupied[i];
                    boxes.push_back(b);
                }

                ROS_ERROR("Now there are %ld boxes in memory", boxes.size());

            }
            else if (feedback->menu_entry_id==7) //delete all
            {
                boxes.erase(boxes.begin(), boxes.end());

            }




            break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            ROS_INFO_STREAM( s.str() << ": pose changed"
                    << "\nposition = "
                    << feedback->pose.position.x
                    << ", " << feedback->pose.position.y
                    << ", " << feedback->pose.position.z
                    << "\norientation = "
                    << feedback->pose.orientation.w
                    << ", " << feedback->pose.orientation.x
                    << ", " << feedback->pose.orientation.y
                    << ", " << feedback->pose.orientation.z
                    << "\nframe: " << feedback->header.frame_id
                    << " time: " << feedback->header.stamp.sec << "sec, "
                    << feedback->header.stamp.nsec << " nsec" );

            if (feedback->marker_name == "CENTER")
            {
                //tf::Transform world_to_center;
                //world_to_center.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
                //world_to_center.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));


                //tf::Transform center_to_corner;
                //center_to_corner.setOrigin(tf::Vector3(target_volume.getSizeX()/2, target_volume.getSizeY()/2,target_volume.getSizeZ()/2));
                //center_to_corner.setRotation(tf::Quaternion(0, 0 , 0 ,1));

                //tf::Transform world_to_corner = world_to_center * center_to_corner;




                //target_volume.setBoundingBox(pose.position.x - target_volume.getSizeX()/2, pose.position.x + target_volume.getSizeX()/2, pose.position.y - target_volume.getSizeY()/2, pose.position.y + target_volume.getSizeY()/2, pose.position.z - target_volume.getSizeZ()/2, pose.position.z + target_volume.getSizeZ()/2);


                //ma.markers.push_back(target_volume.getMarkerWithEdges("target_volume", "/map" , color, ++id));
                //marker_pub->publish(ma);



                //ROS_INFO("center x=%f y=%f z = %f", pose.position.x, pose.position.y, pose.position.z);
                //pose.position.x = world_to_corner.getOrigin().x();
                //pose.position.y = world_to_corner.getOrigin().y();
                //pose.position.z = world_to_corner.getOrigin().z();
                //ROS_INFO("corner x=%f y=%f z = %f", pose.position.x, pose.position.y, pose.position.z);
                //server->setPose( "3DOF_MOVE_3D", pose );


                color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = 1;

                target_volume.setBoundingBox(pose.position.x - target_volume.getSizeX()/2, pose.position.x + target_volume.getSizeX()/2, pose.position.y - target_volume.getSizeY()/2, pose.position.y + target_volume.getSizeY()/2, pose.position.z - target_volume.getSizeZ()/2, pose.position.z + target_volume.getSizeZ()/2);


                ma.markers.push_back(target_volume.getMarkerWithEdges("target_volume", "/map" , color, ++id));
                ma.markers.at(ma.markers.size()-1).lifetime = ros::Duration(0);
                marker_pub->publish(ma);



                pose.position.x = pose.position.x - target_volume.getSizeX()/2;
                pose.position.y = pose.position.y - target_volume.getSizeY()/2;
                pose.position.z = pose.position.z - target_volume.getSizeZ()/2;
                server->setPose( "CORNER", pose );
                server->setPose( "MENU", pose );
            }
            else if (feedback->marker_name == "CORNER")
            {
                double dx = abs(pose.position.x - (target_volume.getCenter()).x()) * 2;
                double dy = abs(pose.position.y - (target_volume.getCenter()).y()) * 2;
                double dz = abs(pose.position.z - (target_volume.getCenter()).z()) * 2;

                target_volume.setSize(dx,dy,dz);

                point3d p = target_volume.getCenter();
                p.x() = p.x() - target_volume.getSizeX()/2;
                p.y() = p.y() - target_volume.getSizeY()/2;
                p.z() = p.z() - target_volume.getSizeZ()/2;

                pose.position.x = p.x();
                pose.position.y = p.y();
                pose.position.z = p.z();
                server->setPose( "CORNER", pose );
                server->setPose( "MENU", pose );


                color.r = 1.; color.g = 1; color.b = 0; color.a = 1;
                ma.markers.push_back(target_volume.getMarkerWithEdges("target_volume", "/map" , color, ++id));
                ma.markers.at(ma.markers.size()-1).lifetime = ros::Duration(0);
                marker_pub->publish(ma);



            }



            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
            break;
    }

    server->applyChanges();
}

void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    geometry_msgs::Pose pose = feedback->pose;

    pose.position.x = round(pose.position.x-0.5)+0.5;
    pose.position.y = round(pose.position.y-0.5)+0.5;

    ROS_INFO_STREAM( feedback->marker_name << ":"
            << " aligning position = "
            << feedback->pose.position.x
            << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z
            << " to "
            << pose.position.x
            << ", " << pose.position.y
            << ", " << pose.position.z );

    server->setPose( feedback->marker_name, pose );
    server->applyChanges();
}

double rand( double min, double max )
{
    double t = (double)rand() / (double)RAND_MAX;
    return min + t*(max-min);
}

void saveMarker( InteractiveMarker int_marker )
{
    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}

////////////////////////////////////////////////////////////////////////////////////

void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof, std::string name )
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 0.5;

    int_marker.name = name;
    int_marker.description = "";

    // insert a box
    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    //if ( fixed )
    //{
    //int_marker.name += "_fixed";
    //int_marker.description += "\n(fixed orientation)";
    //control.orientation_mode = InteractiveMarkerControl::FIXED;
    //}

    //if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    //{
    //std::string mode_text;
    //if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
    //if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
    //if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
    //int_marker.name += "_" + mode_text;
    //int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
    //}

    if(show_6dof)
    {
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        //control.name = "rotate_x";
        //control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        //int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        //control.name = "rotate_z";
        //control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        //int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        //control.name = "rotate_y";
        //control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        //int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
        menu_handler.apply( *server, int_marker.name );
}

void makeMenuMarker( const tf::Vector3& position )
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "MENU";
    int_marker.description = "Drag and drop to choose the position and size of the box, Right click for menu";

    InteractiveMarkerControl control;

    control.interaction_mode = InteractiveMarkerControl::MENU;
    control.name = "menu_only_control";

    Marker marker = makeBox( int_marker );
    marker.scale.x /= 10;
    marker.scale.y /= 10;
    marker.scale.z /= 10;

    control.markers.push_back( marker );
    control.always_visible = true;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    menu_handler.apply( *server, int_marker.name );
} 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "interactive_region_definition");
    ros::NodeHandle n;

    std::string path = ros::package::getPath("interactive_region_definition");

    server.reset( new interactive_markers::InteractiveMarkerServer("interactive_region_definition","",false) );

    load_regions();

    ros::Duration(0.1).sleep();



    //menu_handler.insert( "Define as BBox", &processFeedback );
    //menu_handler.insert( "Define as BBox", &processFeedback );
    //menu_handler.insert( "Second Entry", &processFeedback );
    interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Set box as" );
    menu_handler.insert( sub_menu_handle, "free space", &processFeedback );
    menu_handler.insert( sub_menu_handle, "occupied space", &processFeedback );
    interactive_markers::MenuHandler::EntryHandle sub_menu_handle1 = menu_handler.insert( "File" );
    menu_handler.insert( sub_menu_handle1, "Save to file", &processFeedback );
    menu_handler.insert( sub_menu_handle1, "Load from file", &processFeedback );
    menu_handler.insert( sub_menu_handle1, "Delete all", &processFeedback );



    tf::Vector3 position;
    position = tf::Vector3(-0.600168, 2.0095, 1.9937);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true, "CENTER" );

//-0.600168, 2.0095, 1.9937
//-1.06884, 1.08446, 1.7828
    position = tf::Vector3( -1.6, 1, .2);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true, "CORNER" );
    makeMenuMarker( position );

    server->applyChanges();

    nh = (boost::shared_ptr<ros::NodeHandle>) new ros::NodeHandle;

    ros::Timer timer = nh->createTimer(ros::Duration(0.3), drawingCallback);


    marker_pub = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub = nh->advertise<visualization_msgs::MarkerArray>("target_volume", 10);

    marker_pub_boxes = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub_boxes = nh->advertise<visualization_msgs::MarkerArray>("boxes", 10);




    ros::spin();

    server.reset();
}
