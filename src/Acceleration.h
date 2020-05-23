#include "main.h"

class Acceleration{
	public:
		 Acceleration(double _power, double _divisor);
		 double get_curve(double controller_output);
		 double power;
		 double divisor;
};