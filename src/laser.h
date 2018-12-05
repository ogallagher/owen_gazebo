#ifndef LASER_H
#define LASER_H

/*
	Owen Gallagher
	27 noviembre 2018
*/

#include <limits>       // std::numeric_limits

#include "sensor_msgs/LaserScan.h"

using namespace std;

class Laser {
	public:
		static bool finite;
		static double rangeMin; //si se acerca demasiado a la pared, el sensor da valores incorrectos (no ayuda en todos casos, porque tambien algunos valores incorrectos estallan)
		static double noiseMax; //distancia máxima entre medida real y esperada que aún se acepta como parte de la misma frontera continua (detecta brechas y esquinas; discontinuidades)
		static double offsetMax; //distancia maxima entre el punto de cruce de la frontera antes y la frontera despues, y la medida previa (identifica brechas)
		
		double angleUnitDeg;
		double angleUnitRad;
		double angleMin;
		double angleMax;
		double* measures;
		vector<float>::size_type measuresLen;
		double rangeMax;
		
		Laser();
		bool paramsSet();
		void setParams(const sensor_msgs::LaserScan::ConstPtr &data);
		void readMeasures(const sensor_msgs::LaserScan::ConstPtr &data);
};

#endif
