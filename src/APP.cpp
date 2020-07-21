#include <iostream>
#include <vector>
#include <math.h>
#include <numeric>
using namespace std;


int sign(double x) {
  if (x > 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
}

std::vector<double> get_deviation(std::vector<double> headings, int numb){
  int headingsLength = headings.size();
  int prev_heading = 0;
  std::vector<double> all_headings;
  for(int index=numb; index<=headingsLength; index+=numb) {
    double delta_numb = headingsLength-index;
    if(delta_numb < numb){
      index+=delta_numb;
    }
    std::vector<double> heading_range(headings.begin()+prev_heading,headings.begin()+index);
    int heading_range_length = heading_range.size();
    prev_heading = index;

    double mean = accumulate(heading_range.begin(), heading_range.end(), 0)/heading_range_length; 
    std::vector<double> varianceList;
    for (int headingIndex=0; headingIndex<varianceList.size(); headingIndex++){
      varianceList.push_back(pow(abs(heading_range[headingIndex]), 2));
    }
    double variance = sqrt(accumulate(varianceList.begin(), varianceList.end(), 0)/heading_range_length);
    for (int varianceIndex=0; varianceIndex<heading_range_length; varianceIndex++){
      double point = heading_range[varianceIndex];
      if(abs(point) > variance){
        all_headings.push_back(true);
      } else {
        all_headings.push_back(false);
      }
    }
  }
  return all_headings; 
}


double get_degrees(std::vector<double >point1, std::vector<double > point2) {
  double slope = (point2[1]-point1[1])/(point2[0]-point1[0]);
  double degrees = -atan(slope)*180/3.1415;
  return degrees;
}

double distance(double x1, double y1, double x2, double y2){
  return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
}

std::vector<double> get_intersection(std::vector<double> start, std::vector<double>end, std::vector<double> cur, double radius){
  std::vector<double> p1{start[0]-cur[0], start[1]-cur[1]};
  std::vector<double> p2{end[0]-cur[0], end[1]-cur[1]};
  double dx = p2[0] - p1[0];
  double dy = p2[1] - p1[1];
  double d = sqrt(dx*dx + dy*dy);
  double D = p1[0] * p2[1] * p2[0] * p1[1];

  double discriminant = abs(pow(radius, 2)*pow(d, 2)-pow(D, 2));

  double x1 = (D * dy + sign(dy) * dx * sqrt(discriminant) / pow(d, 2));
  double y1 = (-D * dy + abs(dy) * sqrt(discriminant) / pow(d, 2));
  double x2 = (D * dy - sign(dy) * dx * sqrt(discriminant) / pow(d, 2));
  double y2 = (-D * dy - abs(dy) * sqrt(discriminant) / pow(d, 2));

  double distance1 = distance(p2[0], p2[1], x1, y1);
  double distance2 = distance(p2[0], p2[1], x2, y2);

  std::vector<double> calc1{x1+cur[0], y1+cur[1]};
  std::vector<double> calc2{x2+cur[0], y2+cur[1]};
  if (distance1 < distance2 || distance1 == distance2){return calc1;}
  else if(distance1 > distance2){return calc2;}
}



int main() {
  std::vector<double> path;
  std::vector<double> cur{0.2, 0.2};
  int radius = 1;
  double step = .1;

  int batch_size = 25;
  std::vector<double> all_degrees;

  std::vector<std::vector<double>> points{{1,1},{1,4},{3,5},{4,3},{4,1},{6,1.1}};
  std::vector<double> end_point;

  for(int index=0;index<points.size();index++) {
    std::vector<double> start = points[index];
    std::vector<double> end = points[index+1];
    while(distance(cur[0], cur[1], end[0], end[1]) > radius){
      std::vector<double> new_end = get_intersection(start, end, cur, radius);  
      std::vector<double> new_cur = get_intersection(start, new_end, cur, step);  
      cur = new_cur;  
      double degrees = get_degrees(new_end, cur);
      all_degrees.push_back(degrees);
    }
  }
  std::vector<double> deviation = get_deviation(all_degrees, batch_size);
  cout << deviation[0];
  return 0;
}