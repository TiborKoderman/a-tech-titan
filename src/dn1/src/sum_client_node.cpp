#include "ros/ros.h"
#include "dn1/Sum.h"

// #include <cstdlib>
#include <sstream>
#include <stdlib.h> /* srand, rand */
#include <time.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sum_client_node");
  ros::NodeHandle n;

  srand(time(NULL));

  ros::ServiceClient client = n.serviceClient<dn1::Sum>("sum_service/array");

  dn1::Sum srv;
  std::stringstream ss;

  ss << "[";

  srv.request.arr = {};

  for (int i = 0; i < 10; i++)
  {
    int tmp = rand() % 10 + 1;
    srv.request.arr.push_back(tmp);
    ss << tmp << ", ";
  }

  ss << "]\n";

  ros::service::waitForService("sum_service/array", 1000);

  //   ROS_INFO("Sending: %s", srv.request.content.c_str());

  if (client.call(srv))
  {
    ROS_INFO("The request was: %s, The service returned: %d", ss.str().c_str(), srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}