#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_to_predetermined_points");

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    // actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("move_to_predetermined_points", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    std::ifstream ifs("src/dn3/tocke.json");

    if(!ifs.is_open()){
        // std::cout << "Error opening file" << std::endl;
        ROS_ERROR("Error opening file");
        return 1;
    }

    Json::Value root;
    ifs >> root;

    for(const auto& point : root) {
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = point["point"]["x"].asDouble();
        goal.target_pose.pose.position.y = point["point"]["y"].asDouble();
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal (%f, %f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        ac.sendGoal(goal);

        ac.waitForResult();

        bool finished_before_timeout = ac.waitForResult(ros::Duration(50.0));
        
        // while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && finished_before_timeout){
        //     ROS_INFO("Waiting for goal to be reached");
        // }

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else{
            ROS_INFO("Action did not finish before the time out.");
            ac.cancelGoal();
        }

    }

    return 0;
    
}