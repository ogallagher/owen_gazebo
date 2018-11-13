/*

    Autor: Owen Gallagher
    Fecha de creacion: 6 noviembre 2018
    Descripcion: nodo cliente para controlar el Pioneer3at simulado con Gazebo.

*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>
#include <string>
#include <thread>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_a 0x61
#define KEYCODE_d 0x64
#define KEYCODE_s 0x73
#define KEYCODE_w 0x77

// Generar comando para mover el robot
geometry_msgs::Twist moveRobot(double vel, double ang) {
    geometry_msgs::Twist cmd;
    
    cmd.linear.x = vel; // Eje X es el vector de avance
    cmd.linear.y = 0;
    cmd.linear.z = 0;

    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = ang; // Eje Z es el vector de rotacion

    return cmd;
}

// Escuchar entradas de usuario
void getCommands(ros::Publisher *publisherCmdPtr, ros::Rate *pacemakerPtr) {
    geometry_msgs::Twist cmdMsg;
    char input;
    
    while (ros::ok) {
        ROS_INFO_STREAM("WSAD to move.");
        
        std::cin >> input;
        std::cin.ignore(50,'\n');

        switch(input) {
            case KEYCODE_w:
                ROS_INFO_STREAM("FORWARD");
                cmdMsg = moveRobot(-1,0);
                break;
            case KEYCODE_s:
                ROS_INFO_STREAM("BACKWARD");
                cmdMsg = moveRobot(1,0);
                break;
            case KEYCODE_a:
                ROS_INFO_STREAM("LEFT");
                cmdMsg = moveRobot(0,-1);
                break;
            case KEYCODE_d:
                ROS_INFO_STREAM("RIGHT");
                cmdMsg = moveRobot(0,1);
                break;
            default:
                cmdMsg = moveRobot(0,0);
        }

        (*publisherCmdPtr).publish(cmdMsg);

        (*pacemakerPtr).sleep();
    }
}

// Reaccionar a info del laser
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &data) {
	ROS_INFO_STREAM("Laser scan received!");
}

int main(int argc, char* argv[]) {
    /*
        Necesario para cualquier nodo de ROS.
        Primero se pasa los argumentos de ejecucion, y luego el nombre del nodo
    */
    ros::init(argc,argv,"owen_gazebo_client");

    // Punto de acceso para comunicacion de ROS
    ros::NodeHandle nodeHandle;

    // El nombre del topico que abre owen_gazebo.launch
    ros::Publisher publisherCmd = nodeHandle.advertise<geometry_msgs::Twist>("/Pioneer3AT/cmd_vel",500);
    
    // Este nodo corre un maximo de 2 ciclos cada segundo
    ros::Rate pacemaker(2);

    int counter = 0;

    // Maneja comandos del usuario en hilo separado
    std::thread commandThread(getCommands, &publisherCmd, &pacemaker);
    commandThread.join();

    // Escucha info de laser
    ros::Subscriber subscriberLaser = nodeHandle.subscribe<sensor_msgs::LaserScan>("/Pioneer3AT/laserscan",50,laserCallback);
    
    ROS_INFO_STREAM("owen_gazebo_client is running!");
    while(ros::ok) {
        ros::spinOnce(); //Ejecutar callbacks que estan esperando mensajes (ninguno por ahora)
        counter++;
        pacemaker.sleep(); //Solo revisar topico del laser segun el pacemaker
    }
    ROS_INFO_STREAM("owen_gazebo_client died.");
}
