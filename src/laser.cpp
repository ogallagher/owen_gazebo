/*
	Owen Gallagher
	27 noviembre 2018
*/

#include "laser.h"

using namespace std;

bool Laser::finite = true;
double Laser::noiseMin = 0.1;
double Laser::noiseMax = 25;

Laser::Laser() {
	rangeMax = 4;				//rango de sensado
}

bool Laser::paramsSet() {
	if (!angleUnit || !angleMin || !angleMax) {
		return false;
	}
	else {
		return true;
	}
}

void Laser::setParams(const sensor_msgs::LaserScan::ConstPtr &data) {
	angleUnit = data->angle_increment;
	angleMin = data->angle_min;
	angleMax = data->angle_max;
	
	measuresLen = data->ranges.size();
	measures = new double[measuresLen];
	
	//este codigo no debe usarse; solo es para evitar errores de variables no definidos
	if (!rangeMax) {
		rangeMax = data->range_max;
	}
}

void Laser::readMeasures(const sensor_msgs::LaserScan::ConstPtr &data) {
	for (int i=0; i<measuresLen; i++) {
		double measure = data->ranges[i];
		
		if (measure == numeric_limits<double>::infinity() || measure > rangeMax) {
			measures[i] = numeric_limits<double>::infinity();
		}
		else if (measure < 0.4) { //el sensor da valores extranos cuando se acerca demasiado a la pared
			measures[i] = 0;
		}
		else {
			//normalizar
			//measure = measure / rangeMax;
			
			measures[i] = measure;
		}
	}
}
