
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int16MultiArray.h"


int Arr[2];
void arrayCallback(const std_msgs::Int16MultiArray::ConstPtr& array);

int main(int argc, char** argv)
{

	ros::init(argc, argv, "arraySub");
	ros::NodeHandle n;
	ros::Subscriber arraysub = n.subscribe("array", 100, arrayCallback);
	
	printf("hello");

	for(int j = 0; j < 2; j++)
	{
		printf("%d, ", Arr[j]);
	}

	printf("\n");
	
	ros::spin();
	return 0;

}

void arrayCallback(const std_msgs::Int16MultiArray::ConstPtr& array)
{

	int i = 0;
	// print all the remaining numbers
	for(i; i<array->data.size();++i)
	{
		Arr[i] = array->data.at(i);
	
	}
		for(int j = 0; j < 2; j++)
	{
		printf("%d, ", Arr[j]);
	}

	printf("\n");

	return;
}
