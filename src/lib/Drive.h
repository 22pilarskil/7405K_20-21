#include "main.h"

void rollers(int coefficient, bool flip, std::string powered);
void mecanum(int power, int strafe, int turn);
void brake(std::string mode);
void move_to(double new_y, double new_x, double heading, bool pure_pursuit, double scale, int coefficient, bool flip, std::string powered);
void move_to_pure_pursuit(std::vector<std::vector<double>> points, double scale, int coefficient, bool flip, std::string powered);
