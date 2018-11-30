/*

Owen Gallagher
28 noviembre 2018
Robótica 1

*/

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <SDL2/SDL.h>
#include <cmath>
#include <iostream>
#include <thread>

#include "laser.h"
#include "gnt_graph.h"
#include "drawer.h"

using namespace std;

string laserTopic = "pioneer3dx/laser_scan";

Laser laser;

// Reaccionar a info del laser
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &data) {
	if (!laser.paramsSet()) {
		laser.setParams(data);
		ROS_INFO_STREAM("laser.measuresLen = " + to_string(laser.measuresLen));
	}
	
	int forward = laser.measuresLen / 2;
	laser.readMeasures(data);
	
	string laserConfirm = "laser.measures[" + to_string(forward) + "] = " + to_string(laser.measures[forward]);
	
	ROS_INFO_STREAM(laserConfirm);
}

void readLaser(ros::Rate* pacemakerPtr) {
	ROS_INFO_STREAM("Began readLaser thread.");
    	
	while(ros::ok) {
        ros::spinOnce(); //Ejecutar callbacks que estan esperando mensajes
        pacemakerPtr->sleep(); //Solo revisar topico del laser segun el pacemaker
    }
}

int main(int argc, char* argv[]) {
	/*
        Necesario para cualquier nodo de ROS.
        Primero se pasa los argumentos de ejecucion, y luego el nombre del nodo
    */
    ros::init(argc,argv,"owen_explorer_grapher");
    ros::NodeHandle rosnode;
    ros::Rate pacemaker(2);
    
    //Inicializar lector del sensor
    ros::Subscriber subscriberLaser = rosnode.subscribe<sensor_msgs::LaserScan>(laserTopic,50,laserCallback);
    
    Drawer drawer;
    
	if (drawer.init()) {
		//escuchar topico del laser
		thread laserThread(readLaser, &pacemaker);
		
		//crear GNT
		GNTGraph gntGraph(drawer.width/2,drawer.height/2);
		
		drawer.setColor(0x00,0x00,0xAA,0xFF); //dibujar en azul
		
		double theta = 0;

		while (!drawer.quit && ros::ok) {
			drawer.clear();
			
			while(SDL_PollEvent(&drawer.event) != 0) {
				if (drawer.event.type == SDL_QUIT) {
					drawer.quit = true;
				}
			}
			
			gntGraph.root->moveTo(drawer.width/2 + sin(theta)*30,gntGraph.root->y);
			drawer.graph(&gntGraph);

			drawer.render();
			theta += M_PI*0.0001;
		}
		
		ROS_INFO_STREAM("owen_explorer_grapher killed.");
		return EXIT_SUCCESS;
	}
	else {
		ROS_INFO_STREAM("owen_explorer_grapher killed.");
		return EXIT_FAILURE;
	}
}
