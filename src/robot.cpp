/*
	Owen Gallagher
	27 noviembre 2018
*/

#include "robot.h"

using namespace std;

Robot::Robot(string commandTopic) {
	radius = 1;
	linearMax = 3;
	angularMax = 2;
	linearSpeed = 0.1;
	angularSpeed = 0.1;
	
	this->commandTopic = commandTopic;
}

// El nombre del topico que abre owen_gazebo.launch
void Robot::openPublisher(ros::NodeHandle *nodePtr) {
	publisher = nodePtr->advertise<geometry_msgs::Twist>(commandTopic,500);
}

// Generar comando para mover el robot, y enviarlo
void Robot::move(double vel, double ang) {
	geometry_msgs::Twist cmd;
    
    cmd.linear.x = vel*linearSpeed*linearMax; // Eje X es el vector de avance
    cmd.linear.y = 0;
    cmd.linear.z = 0;

    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = ang*angularSpeed*angularMax; // Eje Z es el vector de rotacion
    
	publisher.publish(cmd);
}
