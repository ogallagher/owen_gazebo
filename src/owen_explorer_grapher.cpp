/*

Owen Gallagher
28 noviembre 2018
Robótica 1

*/

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <SDL2/SDL.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <thread>

#include "owen_gazebo/gnt_raw.h" //generado automaticamente segun msg/gnt_raw.msg
#include "owen_constants.h"
#include "laser.h"
#include "gnt_graph.h"
#include "drawer.h"
#include "gnt.h"

using namespace std;

string gntRawTopic = "pioneer3dx/gnt_raw";
int gntRawLength = -1;
vector<uint8_t> gntRaw;
bool updatingGNT = false;
GNT gntPrevious;
GNT gntCurrent;
GNT gntPreviousPruned; //los GNT recortados no tienen nodos que representan fronteras y otras cosas
GNT gntCurrentPruned;

void updateGNT() {
	gntPrevious.nodes = gntCurrent.nodes; //con vectores en c++, esto es una copia profunda
	gntPreviousPruned.nodes = gntCurrentPruned.nodes;
	
	gntCurrent.update(&gntRaw); //agregar nodos al gnt sin comparar con gntPrevious
	gntCurrent.prune(&gntCurrentPruned);
	
	//analizar cambios entre gntPrevious y gntCurrent
	//TODO: usar este ciclo para tambien ignorar arboles que obviamente son resultados de medidas erroneas
	//		(si hay demasiado cambio en un arbol, probablemente ese arbol no es correcto)
	vector<int> changes; //indices de union, division, aparicion, desaparicion
	bool foundChanges = false;
	
	if (gntCurrentPruned.nodes.size() == gntPreviousPruned.nodes.size()) {
		for (int i=0; i<gntCurrentPruned.nodes.size(); i++) {
			if (gntCurrentPruned.nodes[i].type != gntPreviousPruned.nodes[i].type) {
				changes.push_back(i);
				foundChanges = true;
			}
		}
	}
	else {
		foundChanges = true;
	}
	
	if (foundChanges) {
		//ROS_INFO_STREAM("Found changes!");
		
		if (Laser::finite) {
			//TODO: implementar según los apuntes en realtimeboard
		}
		else {
			//TODO: supongo que esta parte será usada primero, porque aunque he pensado más en la primera,
			//		el caso de rango infinito debería ser más sencillo.
			//		Implementar según los apuntes en realtimeboard
		}
	}
	
	updatingGNT = false;
}

// Reaccionar a info de /gnt_raw
void gntRawCallback(const owen_gazebo::gnt_raw::ConstPtr &msg) {
	if (gntRawLength == -1) {
		gntRawLength = msg->length;
		//ROS_INFO_STREAM("gntRawLength = " + to_string(gntRawLength));
	}
	
	if (!updatingGNT) { //no queremos cambiar gntRaw mientras que updateGNT lo esta usando
		gntRaw = msg->labels;
		
		updatingGNT = true;
		updateGNT();
	}
	else {
		ROS_INFO_STREAM("New raw gnt data received while processing the previous; ignoring it.");
	}
}

void readGNTRaw(ros::Rate* pacemakerPtr) {
	ROS_INFO_STREAM("Began readGNTRaw thread.");
    	
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
    ros::Subscriber subscriberGNTRaw = rosnode.subscribe<owen_gazebo::gnt_raw>(gntRawTopic,50,gntRawCallback);
    
    Drawer drawer;
    
	if (drawer.init()) {
		//escuchar topico del laser
		thread gntRawThread(readGNTRaw, &pacemaker);
		
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
			
			gntGraph.update(&gntCurrent);
			
			drawer.graph(&gntGraph);
			drawer.render();
		}
		
		ROS_INFO_STREAM("owen_explorer_grapher killed.");
		return EXIT_SUCCESS;
	}
	else {
		ROS_INFO_STREAM("owen_explorer_grapher killed.");
		return EXIT_FAILURE;
	}
}
