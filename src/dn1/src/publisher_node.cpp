//Publishing randomly generated velocity messages for turtle-simulator

#include <ros/ros.h> // As previously said the standard ROS library
#include "dn1/Message1.h"
#include <string>

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"dn1_custom_publisher"); //Initialize the ROS system, give the node the name "publish_velocity"
	ros::NodeHandle nh;	//Register the node with the ROS system

	//create a publisher object.
	ros::Publisher pub = nh.advertise<dn1::Message1>("pubsub/chat1", 1000);	//the message tyoe is inside the angled brackets
																						//topic name is the first argument
																						//queue size is the second argument
	//Loop at 1Hz until the node is shutdown.
	ros::Rate rate(1);

    int count=0;
	while(ros::ok()){
		//Create the message.
		dn1::Message1 msg;

		std::string ss = "custom message";

		msg.content = ss;
		msg.seq=count++;

		//Publish the message
		pub.publish(msg);

		//Send a message to rosout
		ROS_INFO("[%d] %s",msg.seq, msg.content);

		//Wait untilit's time for another iteration.
		rate.sleep();
	}

	return 0;
}
