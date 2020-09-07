#include "main.h"
#include "Filter.h"
#include <iostream>
#include <vector>
#include <numeric>
#include <cmath>
using namespace std;

Filter::Filter(double _mean0, double _var0, const float _meanSensor, const float _varSensor, const float _meanMove, const float _varMove, vector<float> _positions, vector<float> _distances)
:	mean0( _mean0), //this is called an initializer list. It assigns values to the variables on creation, which is better
	var0( _var0),
	meanSensor( _meanSensor),
	varSensor( _varSensor),
	meanMove( _meanMove),
	varMove( _varMove),
	positions( _positions),
	distances( _distances){
}
void predict(double &var, double &mean, double &varMove, double &meanMove){
	// += is a little better
	// You probably want to pass by reference, but It wont make that much of a difference (compiler will prob inline it)
	var += varMove;
	mean += meanMove;
//	return vector<double>{var, mean};
}

void correct(double &var0, double &mean0, double &varSensor, double &meanSensor){
	//You originally had the following variables as floats, then returned them in an array of doubles.
	//Since as soon as this function is done, the local variables will be trashed, double may be simpler, because
	//there is no casting required. If you want them to be floats, return vector<float>
	mean0 = (((varSensor * mean0) + (var0 * meanSensor)) / (var0 + varSensor));
	var0 = (1 / (1 / var0 + 1 / varSensor));
//	return vector<double>{newVar, newMean};
}

vector<double> Filter::get_prediction(){
	predict(var0, mean0, varMove, meanMove);
    correct(var0, mean0, varSensor, meanSensor);
//
	vector<vector<double>> vals; //Preset the size (whats the maximum possible size in the end)?
	for (int i = 0; i < positions.size(); i++)
	{
		predict(var0, mean0, varMove, distances[i]);
		correct(var0, mean0, varSensor, positions[i]);
		vals.push_back(vector<double> {var0, mean0});
	}
	return vals[-1];
};

/* OLD CODE
#include "main.h"
#include "filter.h"
#include <iostream>
#include <vector>
#include <numeric>
#include <cmath>
using namespace std;

Filter::Filter(int a, int b, float c, float d, float e, float f, vector<float> g, vector<float> h){
	mean0 = a;
	var0 = b;
	meanSensor = c;
	varSensor = d;
	meanMove = e;
	varMove = f;
	positions = g;
	distances = h;

}

vector<double> Filter::predict(double var, double mean, double varMove, double meanMove){
	double newVar = var+varMove;
	double newMean = mean+meanMove;
	vector<double> returnVals{newVar, newMean};
	return returnVals;
}

vector<double> Filter::correct(double var, double mean, double varSensor, double meanSensor){
	float newMean = ((varSensor*mean) + (var*meanSensor)) / (var+varSensor);
	float newVar  = 1/(1/var + 1/varSensor);
	vector<double> returnVals{newVar, newMean};
	return returnVals;
}

vector<double> Filter::get_prediction(){
	vector<double> new_var = predict(var0, mean0, varMove, meanMove);
	vector<double> all_vals = correct(new_var[0], new_var[1], varSensor, meanSensor);
	vector<vector<double>> vals;
	for(int count=0; count < positions.size(); count++){
		all_vals = predict(all_vals[0], all_vals[1], varMove, distances[count]);
		all_vals = correct(all_vals[0], all_vals[1], varSensor, positions[count]);
		vals.push_back(all_vals);
	}
	return vals[-1];
};
*/