/*

    Autor: Owen Gallagher
    Fecha de creacion: 6 noviembre 2018
    Descripcion: nodo cliente para controlar el Pioneer3at simulado con Gazebo.

*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>
#include <thread>
#include <limits>       // std::numeric_limits

#include "laser.h"
#include "robot.h"
#include "gnt_graph.h"
#include "drawer.h"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_a 0x61
#define KEYCODE_d 0x64
#define KEYCODE_s 0x73
#define KEYCODE_w 0x77

using namespace std;

// Parametros
string robotNS = "pioneer3dx";
string topicCmd = "/cmd_vel";
string topicLaser = "/laser_scan";
string topicGNT = "/gnt_raw"

//Objetos
Laser laser;
Robot robot(robotNS + topicCmd);
ros::Publisher* publisherGNT = NULL;

// Escuchar entradas de usuario
void getCommands(ros::Rate *pacemakerPtr) {
	char input;
	
	while (ros::ok) {
	    ROS_INFO_STREAM("WSAD to move. Anything else to stop.");
	    
	    cin >> input;
	    cin.ignore(50,'\n');

	    switch(input) {
	        case KEYCODE_w:
	            ROS_INFO_STREAM("FORWARD");
	            robot.move(1,0);
	            break;
	            
	        case KEYCODE_s:
	            ROS_INFO_STREAM("BACKWARD");
	            robot.move(-1,0);
	            break;
	            
	        case KEYCODE_a:
	            ROS_INFO_STREAM("LEFT");
	            robot.move(0,-1);
	            break;
	            
	        case KEYCODE_d:
	            ROS_INFO_STREAM("RIGHT");
	            robot.move(0,1);
	            break;
	            
	        default:
	            robot.move(0,0);
	    }
    }
}

// Reaccionar a info del láser
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &data) {
	if (!laser.paramsSet()) {
		laser.setParams(data);
		ROS_INFO_STREAM("laser.measuresLen = " + to_string(laser.measuresLen));
	}
	
	int forward = laser.measuresLen / 2;
	laser.readMeasures(data);
	
	string laserConfirm = "laser.measures[" + to_string(forward) + "] = " + to_string(laser.measures[forward]);
	
	if (publisherGNT != NULL) {
		OwenConstants::NodeType labels[laser.measuresLen];
		
		for (int i=0; i<laser.measuresLen; i++) {
			labels[i] = OTHER;
		}
		
		publisherGNT->publish(labels);
	}
	
	ROS_INFO_STREAM(laserConfirm);
}

void readLaser(ros::Rate *pacemakerPtr) {
	int counter = 0;
	
	while(ros::ok) {
    	//ROS_INFO_STREAM("laserThread: spin_" + to_string(counter));
    	
        ros::spinOnce(); //Ejecutar callbacks que estan esperando mensajes
        
        counter++;
        
        pacemakerPtr->sleep(); //Solo revisar topico del láser segun el pacemaker
    }
}

int main(int argc, char* argv[]) {
    /*
        Necesario para cualquier nodo de ROS.
        Primero se pasa los argumentos de ejecucion, y luego el nombre del nodo
    */
    ros::init(argc,argv,"owen_gazebo_client");

    // Punto de acceso para comunicacion de ROS
    ros::NodeHandle nodeHandle;

    robot.openPublisher(&nodeHandle);
    
    // Este nodo corre un maximo de 2 ciclos cada segundo
    ros::Rate pacemaker(2);

    // Maneja comandos del usuario en hilo separado
    thread commandThread(getCommands, &pacemaker);

    // Escucha info de laser
    string laserTopic = robotNS + topicLaser;
    ros::Subscriber subscriberLaser = nodeHandle.subscribe<sensor_msgs::LaserScan>(laserTopic,50,laserCallback);
    
    thread laserThread(readLaser, &pacemaker);
	
	//Publica a gnt_raw
	string gntTopic = robotNS + topicGNT;
	publisherGNT = nodeHandle.advertise<gnt_raw>(gntTopic,500);
    
    ROS_INFO_STREAM("owen_gazebo_client born.");
    
    while (ros::ok) {
    	//nothing
    }
    
    ROS_INFO_STREAM("owen_gazebo_client killed.");
	
	return 0;
}
