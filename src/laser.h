#ifndef LASER_H
#define LASER_H

/*
	Owen Gallagher
	27 noviembre 2018
*/

#include <math.h>
#include <limits>       // std::numeric_limits

#include "sensor_msgs/LaserScan.h"

using namespace std;

class Laser {
	public:
		double angleUnit;
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
