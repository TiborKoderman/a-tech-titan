#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <vector>

int main(int argc, char **argv){
    ros::init(argc, argv, "save_location");
    ros::NodeHandle nh;

    //publish marker at current position
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    visualization_msgs::Marker marker;
    

    ros::Rate r(1);
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // marker.lifetime = ros::Duration();

    while(ros::ok()){
    ROS_INFO("Publishing marker");
    marker_pub.publish(marker);
    ros::spinOnce();
    }

    

    return 0;
}