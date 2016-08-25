#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sstream>
#define NODE_NAME "Listener"

float received=0; 

void listenResponse(const std_msgs::Float64::ConstPtr& fmsg)
{
	received=fmsg->data;
	ROS_INFO("I heard: %f",fmsg->data);
}



int main(int argc, char **argv)
{

	ros::init(argc, argv,"subpub"); //init ros for this node
	
	ros::NodeHandle nodeHandle;       //object that handles communication and fully initialize this node

	ros::Subscriber sub = nodeHandle.subscribe("Ultrasound", 1000, listenResponse); 
																																															 
	ros::Publisher chatter_pub = nodeHandle.advertise<std_msgs::String>("UltrasonicResponse", 1000);
	
	ros::Rate loop_rate(10);
	  
	while(ros::ok())
	{
	
	std_msgs::String msg;
	std::stringstream ss;
	ss<<"Publishing: "<<received;
	msg.data=ss.str();
	chatter_pub.publish(msg);
//	ROS_INFO("I heard while publishing: %f",received);
	loop_rate.sleep();
	ros::spinOnce();
	
	}
	


}



