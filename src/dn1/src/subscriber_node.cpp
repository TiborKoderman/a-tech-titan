#include <ros/ros.h>
#include "dn1/Message1.h" //The message type

//This is a callback function. It executes every time a new pose message arrives
void messageReceived(const dn1::Message1::ConstPtr& msg){	//The parameter is the message type

	ROS_INFO("[%d] %s",msg->seq, msg->content.c_str());
}

int main(int argc, char **argv){

	ros::init(argc, argv,"our_custom_subscriber");
	ros::NodeHandle nh;

	//Create a subscriber objet
	ros::Subscriber sub=nh.subscribe("pubsub/chat1",1000,&messageReceived);	//First parameter is the topic
																				//Second parameter is the queue size
																				//Third parameter is a ponter to a
																					//callback function that will execute
																					//each time a new message is recieved

	//Ros will take over after this
	ros::spin();
	//the same thing can be done with a loop like:
		//while(ros::ok()){  ros::spinOnce(); }
}
