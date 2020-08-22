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