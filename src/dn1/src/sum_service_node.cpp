#include "ros/ros.h"
#include "dn1/Sum.h"

#include <string>
#include <iostream>
#include <algorithm>
#include <vector>

bool manipulate(dn1::Sum::Request& req, dn1::Sum::Response& res)
{
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sum_service_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("sum_service/array", manipulate);
  ros::spin();

  return 0;
}
