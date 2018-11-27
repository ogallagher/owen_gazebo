#ifndef LASER_H
#define LASER_H

#include <math.h>

#include "sensor_msgs/LaserScan.h"

using namespace std;

class Laser {
	public:
		double angleUnit;
		double angleMin;
		double angleMax;
		double* measures;
		int measuresLen;
		
		Laser();
		bool paramsSet();
		void setParams(const sensor_msgs::LaserScan::ConstPtr &data);
		void readMeasures(const sensor_msgs::LaserScan::ConstPtr &data);
};

#endif
