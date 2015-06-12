#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

#include <world_model_consistency_check/bounding_box.h>

using namespace visualization_msgs;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
boost::shared_ptr<ros::NodeHandle> nh;
boost::shared_ptr<ros::Publisher> marker_pub;
boost::shared_ptr<ros::Publisher> marker_pub_boxes;

ClassBoundingBox target_volume(0, 2, 0, 2, 0, 2);
std::vector<ClassBoundingBox> boxes;


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
    color.r = .0; color.g = 1.0; color.b = 1; color.a = 1;
    std_msgs::ColorRGBA color_free;
    color_free.r = 1.0; color_free.g = 1.0; color_free.b = 0; color_free.a = 1;

    size_t id = 0;

    for (size_t i = 0; i< boxes.size(); ++i)
    {
        if (boxes[i].occupied == true)
            ma.markers.push_back(boxes[i].getMarkerWithEdges("boxes", "/base_link" , color, ++id));
        else
            ma.markers.push_back(boxes[i].getMarkerWithEdges("boxes", "/base_link" , color_free, ++id));

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


                //ma.markers.push_back(target_volume.getMarkerWithEdges("target_volume", "/base_link" , color, ++id));
                //marker_pub->publish(ma);



                //ROS_INFO("center x=%f y=%f z = %f", pose.position.x, pose.position.y, pose.position.z);
                //pose.position.x = world_to_corner.getOrigin().x();
                //pose.position.y = world_to_corner.getOrigin().y();
                //pose.position.z = world_to_corner.getOrigin().z();
                //ROS_INFO("corner x=%f y=%f z = %f", pose.position.x, pose.position.y, pose.position.z);
                //server->setPose( "3DOF_MOVE_3D", pose );


                color.r = .0; color.g = 1.0; color.b = 0; color.a = 1;

                target_volume.setBoundingBox(pose.position.x - target_volume.getSizeX()/2, pose.position.x + target_volume.getSizeX()/2, pose.position.y - target_volume.getSizeY()/2, pose.position.y + target_volume.getSizeY()/2, pose.position.z - target_volume.getSizeZ()/2, pose.position.z + target_volume.getSizeZ()/2);


                ma.markers.push_back(target_volume.getMarkerWithEdges("target_volume", "/base_link" , color, ++id));
                marker_pub->publish(ma);



                pose.position.x = pose.position.x + target_volume.getSizeX()/2;
                pose.position.y = pose.position.y + target_volume.getSizeY()/2;
                pose.position.z = pose.position.z + target_volume.getSizeZ()/2;
                server->setPose( "CORNER", pose );
                server->setPose( "MENU", pose );
            }
            else if (feedback->marker_name == "CORNER")
            {
                double dx = (pose.position.x - (target_volume.getCenter()).x()) * 2;
                double dy = (pose.position.y - (target_volume.getCenter()).y()) * 2;
                double dz = (pose.position.z - (target_volume.getCenter()).z()) * 2;

                target_volume.setSize(dx,dy,dz);

                point3d p = target_volume.getCenter();
                p.x() = p.x() + target_volume.getSizeX()/2;
                p.y() = p.y() + target_volume.getSizeY()/2;
                p.z() = p.z() + target_volume.getSizeZ()/2;

                pose.position.x = p.x();
                pose.position.y = p.y();
                pose.position.z = p.z();
                server->setPose( "CORNER", pose );
                server->setPose( "MENU", pose );


                color.r = .0; color.g = 1.0; color.b = 0; color.a = 1;
                ma.markers.push_back(target_volume.getMarkerWithEdges("target_volume", "/base_link" , color, ++id));
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
// %EndTag(processFeedback)%

// %Tag(alignMarker)%
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
// %EndTag(alignMarker)%

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
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = name;
    int_marker.description = "Simple 6-DOF Control";

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
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
        menu_handler.apply( *server, int_marker.name );
}
// %EndTag(6DOF)%

void makeMenuMarker( const tf::Vector3& position )
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "MENU";
    int_marker.description = "Menu \n (Right click)";

    InteractiveMarkerControl control;

    control.interaction_mode = InteractiveMarkerControl::MENU;
    control.name = "menu_only_control";

    Marker marker = makeBox( int_marker );
    marker.scale.x /= 2;
    marker.scale.y /= 2;
    marker.scale.z /= 2;

    control.markers.push_back( marker );
    control.always_visible = true;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    menu_handler.apply( *server, int_marker.name );
} 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "basic_controls");
    ros::NodeHandle n;

    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

    ros::Duration(0.1).sleep();

    //menu_handler.insert( "Define as BBox", &processFeedback );
    //menu_handler.insert( "Define as BBox", &processFeedback );
    //menu_handler.insert( "Second Entry", &processFeedback );
    interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Define" );
    menu_handler.insert( sub_menu_handle, "Free space", &processFeedback );
    menu_handler.insert( sub_menu_handle, "Occupied space", &processFeedback );

    tf::Vector3 position;
    position = tf::Vector3(-3, 3, 0);
    // make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
    // position = tf::Vector3( 0, 3, 0);
    // make6DofMarker( true, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
    // position = tf::Vector3( 3, 3, 0);
    // makeRandomDofMarker( position );
    // position = tf::Vector3(-3, 0, 0);
    // make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::ROTATE_3D, position, false );
    // position = tf::Vector3( 0, 0, 0);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_3D, position, false, "CENTER" );
    position = tf::Vector3( 3, 0, 0);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_3D, position, false, "CORNER" );
    // position = tf::Vector3(-3,-3, 0);
    // makeViewFacingMarker( position );
    // position = tf::Vector3( 0,-3, 0);
    // makeQuadrocopterMarker( position );
    // position = tf::Vector3( 3,-3, 0);
    // makeChessPieceMarker( position );
    // position = tf::Vector3(-3,-6, 0);
    // makePanTiltMarker( position );
    // position = tf::Vector3( 0,-6, 0);
    // makeMovingMarker( position );
    // position = tf::Vector3( 3,-6, 0);
    makeMenuMarker( position );
    // position = tf::Vector3( 0,-9, 0);
    // makeButtonMarker( position );

    server->applyChanges();

    nh = (boost::shared_ptr<ros::NodeHandle>) new ros::NodeHandle;

    ros::Timer timer = nh->createTimer(ros::Duration(0.3), drawingCallback);


    marker_pub = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub = nh->advertise<visualization_msgs::MarkerArray>("/target_volume", 10);

    marker_pub_boxes = (boost::shared_ptr<ros::Publisher>) (new ros::Publisher);
    *marker_pub_boxes = nh->advertise<visualization_msgs::MarkerArray>("/boxes", 10);




    ros::spin();

    server.reset();
}
