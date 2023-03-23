#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_to_predetermined_points");

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    // actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("move_to_predetermined_points", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 0.8667454123497009;
    goal.target_pose.pose.position.y = -2.0688655376434326;
    // goal.goal.target_pose.pose.position.z = -0.001434326171875;
    goal.target_pose.pose.orientation.w = 1.0;

    // goal.x = 0.8667454123497009;
    // goal.y = -2.0688655376434326;
    // goal.z = -0.001434326171875;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //     ROS_INFO("Goal reached");
    // else
    //     ROS_INFO("Failed to reach goal");


    bool finished_before_timeout = ac.waitForResult(ros::Duration(50.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else{
        ROS_INFO("Action did not finish before the time out.");
        ac.cancelGoal();
    }

    return 0;
    
}