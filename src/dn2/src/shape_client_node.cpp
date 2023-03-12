#include "ros/ros.h"
#include "dn2/Shape.h"

// #include <cstdlib>
#include <string>
#include <iostream>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "shape_client_node");
  ros::NodeHandle n;


  ros::ServiceClient client = n.serviceClient<dn2::Shape>("shape_service/Shape");

  dn2::Shape srv;

  srv.request.shape = argv[1];
  srv.request.duration = std::stoi(argv[2]);

  ROS_INFO("Sending: %s, for duration: %d", srv.request.shape.c_str(), srv.request.duration);

  ros::service::waitForService("shape_service/Shape", 1000);

  //   ROS_INFO("Sending: %s", srv.request.content.c_str());

  if (client.call(srv))
  {
    ROS_INFO("the request was successful");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}