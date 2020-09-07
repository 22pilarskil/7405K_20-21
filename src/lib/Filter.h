#include "main.h"
#include <vector>

class Filter {
private:
    double mean0;
    double var0;
    const double meanSensor;
    const double varSensor;
    const double meanMove;
    const double varMove;
    std::vector<float> positions;
    std::vector<float> distances;
    void predict(double &var, double &mean, double varMove, double meanMove);
    void correct(double &var0, double &mean0, double varSensor, double meanSensor);
public:
    Filter(double _mean0, double _var0, float _meanSensor, float _varSensor, float _meanMove, float _varMove, std::vector<float> _positions, std::vector<float> _distances);
    std::vector<double> get_prediction();
};