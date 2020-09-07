#include "main.h"
#include <vector>
using namespace std;

class Filter {
  private:
    double mean0;
	double var0;
	float meanSensor;
	float varSensor;
	float meanMove;
	float varMove;
	vector<float> positions;
	vector<float> distances;
	vector<double> predict(double var, double mean, double varMove, double meanMove);
	vector<double> correct(double var, double mean, double varSensor, double meanSensor);
  public:            
    Filter(double _mean0, double _var0, const float _meanSensor, const float _varSensor, const float _meanMove, const float _varMove, vector<float> _positions, vector<float> _distances);
	vector<double> get_prediction();
};