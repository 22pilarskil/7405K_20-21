#include "main.h"
#include <vector>
using namespace std;

class Filter {       
  public:             
    int mean0;
	int var0;
	float meanSensor;
	float varSensor;
	float meanMove;
	float varMove;
	vector<float> positions;
	vector<float> distances;
	Filter(int a, int b, float c, float d, float e, float f, vector<float> g, vector<float> h);
	vector<double> get_prediction();
	vector<double> predict(double var, double mean, double varMove, double meanMove);
	vector<double> correct(double var, double mean, double varSensor, double meanSensor); 

};