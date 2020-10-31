#include "PurePursuit.h"
#include <vector>
#include <math.h>
#include <numeric>
#include <cmath>
using namespace pros;

#define SQD(n) pow(n, 2)


/**
 * @return: An integer value (1, -1 or 0) that represents the sign of the input x
 */
int sign(double x) {
  if (x > 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
}

double get_degrees(std::vector<double> p1, std::vector<double> p2) {
  double y = p1[0] - p2[0];
  double x = p1[1] - p2[1];
  return atan2(-x, y) * 180 / M_PI;
}

/**
 * @param cur: A vector of length two in the format {Y, X} that represents our robot's current position
 * @param speeds: Same format as @param speeds from Robot::move_to, see above
 */
double distance(std::vector<double> cur, std::vector<double> end) {
  return sqrt(SQD(cur[0] - end[0]) + SQD(cur[1] - end[1]));
}

std::vector<double> get_intersection(std::vector<double> start, std::vector<double> end, std::vector<double> cur, double radius) {
  std::vector<double> p1 {start[0] - cur[0], start[1] - cur[1]};
  std::vector<double> p2 {end[0] - cur[0], end[1] - cur[1]};

  double dx = p2[0] - p1[0];
  double dy = p2[1] - p1[1];
  float d = sqrt(SQD(dx) + SQD(dy));
  float D = p1[0] * p2[1] - p2[0] * p1[1];
  float discriminant = abs(SQD(radius) * SQD(d) - SQD(D));

  float x1 = (D * dy + sign(dy) * dx * sqrt(discriminant)) / SQD(d);
  float y1 = (-D * dx + abs(dy) * sqrt(discriminant)) / SQD(d);
  float x2 = (D * dy - sign(dy) * dx * sqrt(discriminant)) / SQD(d);
  float y2 = (-D * dx - abs(dy) * sqrt(discriminant)) / SQD(d);

  std::vector<double> intersection1 {x1, y1};
  std::vector<double> intersection2 {x2, y2};

  float distance1 = distance(p2, intersection1);
  float distance2 = distance(p2, intersection2);

  std::vector<double> calc1 {(x1 + cur[0]), (y1 + cur[1])};
  std::vector<double> calc2 {(x2 + cur[0]), (y2 + cur[1]) };

  if (distance1 < distance2) return calc1;
  if (distance1 > distance2) return calc2;
}
