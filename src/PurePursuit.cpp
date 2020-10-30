#include "PurePursuit.h"
#include <vector>
#include <math.h>
#include <numeric>
#include <cmath>
using namespace pros;

#define SQD(n) pow(n, 2)

int sign(double x) {
  if (x > 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
}

float point_line_distance(std::vector<double> p1, std::vector<double> p2, std::vector<double> p3) {
  double dy = p1[1] - p2[1];
  double dx = p1[0] - p2[0];

  if (dy == 0) return abs(p1[1] - p3[1]);
  if (dx == 0) return abs(p1[0] - p3[0]);

  float slope = dy / dx;
  float a = -slope;
  int b = 1;
  float c = slope * p2[0] - p2[1];
  return abs((a * p3[0] + b * p3[1] + c) / sqrt(SQD(a) + SQD(b)));
}

std::vector<double> get_deviation(std::vector<double> headings, int numb)
{
  int headingsLength = headings.size();
  int prev_heading = 0;
  std::vector<double> all_headings;
  for(int index = numb; index <= headingsLength; index += numb) {
    double delta_numb = headingsLength - index;
    if(delta_numb < numb) index += delta_numb;
    
    std::vector<float> heading_range(headings.begin() + prev_heading, headings.begin() + index);
    int heading_range_length = heading_range.size();
    prev_heading = index;

    float mean = accumulate(heading_range.begin(), heading_range.end(), 0) / heading_range_length;

    std::vector<double> varianceList;
    for (int headingIndex = 0; headingIndex < heading_range_length; headingIndex++){
      varianceList.push_back(SQD(heading_range[headingIndex] - mean));
    }
    double variance = sqrt(accumulate(varianceList.begin(), varianceList.end(), 0) / heading_range_length);
    all_headings.push_back(variance);
  }
  return all_headings;
}


double get_degrees(std::vector<double> p1, std::vector<double> p2) {
  double y = p1[0] - p2[0];
  double x = p1[1] - p2[1];
  return atan2(-x, y) * 180 / M_PI;
}

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
