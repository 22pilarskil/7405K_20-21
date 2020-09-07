#include "main.h"

class PID{
	public:
		double kp;
		double ki;
		double kd;
		double minspeed;

		double prev_error;
		int prev_time;
		double I;
		double D;

		PID(double p, double i, double d, double min = 0);
		double get_value(double error);
		void reset();
};