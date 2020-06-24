#include "main.h"

class PID{
	public:
		double kp;
		double ki;
		double kd;
		double prev_error;
		double I;
		double D;
		double minspeed;
		PID(double p, double i, double d, double min = 0);
		double get_value(double error);
		void reset();
};