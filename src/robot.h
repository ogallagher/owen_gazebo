#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace std;

/*
	Owen Gallagher
	27 noviembre 2018
*/

class Robot {		
	public:
		string commandTopic;
		ros::Publisher publisher;
		
		double linearMax;
		double angularMax;
		double linearSpeed;
		double angularSpeed;
		double radius;
		
		Robot(string commandTopic);
		void openPublisher(ros::NodeHandle *nodePtr);
		void move(double vel, double ang);
};

#endif
