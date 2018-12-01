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
#include <vector>

#include "owen_constants.h"
#include "owen_gazebo/gnt_raw.h"
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
string topicGNT = "/gnt_raw";

//Objetos
Laser laser;
Robot robot(robotNS + topicCmd);
ros::Publisher* publisherGNTPtr = NULL;

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
	
	//string confirmation = "laser.measures[" + to_string(forward) + "] = " + to_string(laser.measures[forward]);
	//ROS_INFO_STREAM(confirmation);
	
	//etiquetar medidas y luego enviar etiquetas
	if (publisherGNTPtr != NULL) {
		owen_gazebo::gnt_raw gntMessage;
		vector<uint8_t> labels;
		
		OwenConstants::NodeType currentLabel = OwenConstants::OTHER;
		OwenConstants::NodeType previousLabel = OwenConstants::OTHER;
		OwenConstants::NodeType oldLabel = OwenConstants::OTHER;
		
		double currentMeasure;
		double previousMeasure;
		double oldMeasure;
		double noise;
		
		currentMeasure = laser.measures[laser.measuresLen-1];
		previousMeasure = laser.measures[laser.measuresLen-1-1];
		
		//checar si medidas son brechas o nubes
		for (int i=0; i<laser.measuresLen; i++) {
			//actualizar medidas
			oldMeasure = previousMeasure;
			previousMeasure = currentMeasure;
			currentMeasure = laser.measures[i];
			
			if (currentMeasure == numeric_limits<double>::infinity()) { //nube
				currentLabel = OwenConstants::CLOUD;
				
				if (previousLabel == OwenConstants::WALL) { //inicio de nube
					previousLabel = OwenConstants::GAP;
				}
			}
			else { //brecha o frontera
				if (previousLabel == OwenConstants::CLOUD) {
					currentLabel = OwenConstants::GAP; //fin de nube
				}
				else {
					//determinar ruido (funcion de error)
					noise = oldMeasure - previousMeasure;
					if (noise == 0) {
						noise = laser.noiseMin;
					}
					
					ROS_INFO_STREAM("Noise.denom: " + to_string(noise));
					noise = abs((previousMeasure - currentMeasure) - noise) / abs(noise);
					ROS_INFO_STREAM("Noise: " + to_string(noise));
					
					//determinar si actual es una brecha
					if (noise > Laser::noiseMax) {
						currentLabel = OwenConstants::GAP; //dista demasiado para solo ser ruido
					}
					else {
						currentLabel = OwenConstants::WALL; //probablemente es parte de la misma frontera 
					}
				}
			}
		
			labels.push_back(static_cast<uint8_t>(currentLabel));
		}
		
		gntMessage.length = static_cast<int>(laser.measuresLen);
		gntMessage.labels = labels;
		
		publisherGNTPtr->publish(gntMessage);
	}
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
    ros::NodeHandle rosnode;

    robot.openPublisher(&rosnode);
    
    // Este nodo corre un maximo de 2 ciclos cada segundo
    ros::Rate pacemaker(2);

    // Maneja comandos del usuario en hilo separado
    thread commandThread(getCommands, &pacemaker);
    
    //Publica a gnt_raw
	string gntTopic = robotNS + topicGNT;
	ros::Publisher publisherGNT = rosnode.advertise<owen_gazebo::gnt_raw>(gntTopic,500);
	publisherGNTPtr = &publisherGNT;

    // Escucha info de laser
    string laserTopic = robotNS + topicLaser;
    ros::Subscriber subscriberLaser = rosnode.subscribe<sensor_msgs::LaserScan>(laserTopic,50,laserCallback);
    
    thread laserThread(readLaser, &pacemaker);
    
    ROS_INFO_STREAM("owen_gazebo_client born.");
    
    while (ros::ok) {
    	//nothing
    }
    
    ROS_INFO_STREAM("owen_gazebo_client killed.");
	
	return 0;
}
