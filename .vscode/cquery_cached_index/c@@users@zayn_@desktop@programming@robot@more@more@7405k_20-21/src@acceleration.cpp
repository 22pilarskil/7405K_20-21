#include "Acceleration.h"


Acceleration::Acceleration (double _power, double _divisor){
	power = _power;
	divisor = _divisor;
}

double Acceleration::get_curve(double controller_output){
	double sign = 1;
	if (controller_output < 0){
		sign = -1;
	}
	double curve = std::pow(abs(controller_output / divisor), power);
	return curve * sign;
}
