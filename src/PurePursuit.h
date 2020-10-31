#include "main.h"
#include <vector>

int sign(double x);
float point_line_distance(std::vector<double> p1, std::vector<double> p2, std::vector<double> p3);
double distance(std::vector<double> p1, std::vector<double> p2);
double get_degrees(std::vector<double> p1, std::vector<double> p2);
std::vector<double> get_deviation(std::vector<double> headings, int numb);
std::vector<double> get_intersection(std::vector<double> start, std::vector<double> end, std::vector<double> cur, double radius);

