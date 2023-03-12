#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "dn2/Shape.h"

#include <iostream>
#include <string>

bool manipulate(dn2::Shape::Request& req, dn2::Shape::Response& res)
{
  double scale_linear, scale_angular;
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

  ros::param::get("~scale_linear", scale_linear);
  ros::param::get("~scale_angular", scale_angular);

  double secs = ros::Time::now().toSec();

  int step = 1;
  int side = 0;

  
  while (ros::ok() && ros::Time::now().toSec() - secs < req.duration)
  {
    ros::Rate rate(4);
    geometry_msgs::Twist msg;

    if (req.shape == "circle")
    {
      msg.linear.x = 0.2;
      msg.angular.z = 0.2;
    }
    else if (req.shape == "square")
    {
      msg.linear.x = 0.2;
      step = step % 20;

      if (step % 5 == 0)
      {
        msg.linear.x = 0;
        msg.angular.z = 1.57;  //(90 / 360) * 2 * 3.14
      }
    }
    else if (req.shape == "rectangle")
    {
      msg.linear.x = 0.2;
      step = step % 30;

      if (step % (10 * (side % 2 + 1)) == 0)
      {
        msg.linear.x = 0;
        msg.angular.z = 1.57;  //(90 / 360) * 2 * 3.14
        side++;
      }
    }
    else if (req.shape == "triangle")
    {
      msg.linear.x = 0.2;
      step = step % 15;

      if (step % 5 == 0)
      {
        msg.linear.x = 0;
        msg.angular.z = 1.047;  //(60 / 360) * 2 * 3.14
      }
    }
    else
    {
      res.lastMovementType = "Error: unknown shape";
      return false;
    }
    pub.publish(msg);
    step++;
  }
  res.lastMovementType = req.shape;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "shape_service_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("shape_service/Shape", manipulate);
  ROS_INFO("Service ready");
  ros::spin();

  return 0;
}