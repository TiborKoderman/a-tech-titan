#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "dn2/Shape.h"

#include <iostream>
#include <string>
#include <time.h>
#include <stdlib.h>

bool manipulate(dn2::Shape::Request& req, dn2::Shape::Response& res)
{
  double scale_linear, scale_angular;
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);


  double secs = ros::Time::now().toSec();

  int step = 1;
  int side = 0;

  ros::Rate rate(1);
  while (ros::ok() && ros::Time::now().toSec() - secs < req.duration)
  {
    geometry_msgs::Twist msg;

    if (req.shape == "circle")
    {
      msg.linear.x = 0.2;
      msg.angular.z = 0.2;
    }
    else if (req.shape == "square")
    {
      msg.linear.x = 0.2;
      if (step % 10 == 0)
      {
        msg.linear.x = 0;
        msg.angular.z = 1.57;  //(90 / 360) * 2 * 3.14
      }
    }
    else if (req.shape == "rectangle")
    {
      msg.linear.x = 0.2;

      if (step % (10 * (side % 2 + 1)) == 0)
      {
        msg.linear.x = 0;
        msg.angular.z = 1.57;  //(90 / 360) * 2 * 3.14
        side++;
        step = 0;
      }
    }
    else if (req.shape == "triangle")
    {
      msg.linear.x = 0.2;

      if (step % 10 == 0)
      {
        msg.linear.x = 0;
        msg.angular.z = 2.0933333333333333;  //(60 / 180) * 2 * 3.14
      }
    }
    else if (req.shape == "random")
    {
      msg.linear.x = 0.3 * (double(rand()) / double(RAND_MAX));
      msg.angular.z = 0.5 * 2 * (double(rand()) / double(RAND_MAX) - 0.5);
    }
    else
    {
      res.lastMovementType = "Error: unknown shape";
      return false;
    }
    ROS_INFO("[%d] linear: %f, angular: %f", step, msg.linear.x, msg.angular.z);
    pub.publish(msg);
    step++;
    rate.sleep();
  }
  res.lastMovementType = req.shape;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "shape_service_node");
  ros::NodeHandle n;

  srand(time(NULL));

  ros::ServiceServer service = n.advertiseService("shape_service/Shape", manipulate);
  ROS_INFO("Service ready");
  ros::spin();

  return 0;
}