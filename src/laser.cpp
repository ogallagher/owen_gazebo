/*
	Owen Gallagher
	27 noviembre 2018
*/

#include <cmath>
#include "laser.h"

using namespace std;

bool Laser::finite = true;
double Laser::rangeMin = 0.5;
double Laser::noiseMax = 0.5;
double Laser::offsetMax = 0.03;

Laser::Laser() {
	rangeMax = 4;				//rango de sensado
}

bool Laser::paramsSet() {
	if (!angleUnitDeg || !angleMin || !angleMax) {
		return false;
	}
	else {
		return true;
	}
}

void Laser::setParams(const sensor_msgs::LaserScan::ConstPtr &data) {
	angleUnitDeg = data->angle_increment;
	angleUnitRad = (angleUnitDeg/180.0)*M_PI;
	
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
		else if (measure < rangeMin) { //el sensor da valores extranos cuando se acerca demasiado a la pared
			measures[i] = rangeMin;
		}
		else {
			//normalizar
			//measure = measure / rangeMax;
			
			measures[i] = measure;
		}
	}
}
