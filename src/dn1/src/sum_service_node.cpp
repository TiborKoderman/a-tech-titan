#include "ros/ros.h"

#include <string>
#include <iostream>
#include <algorithm>
#include <vector>

bool manipulate(Vector<int>& req, int res)
{
  res = 0;
  for (int i : req.content)
  {
    res += i;
  }

  ROS_INFO("response: %d", req, res);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sum_service_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("sum_service/array", manipulate);
  ros::spin();

  return 0;
}
