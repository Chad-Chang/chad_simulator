// #ifndef GLOBAL_FUNCTION_H
// #define GLOBAL_FUNCTION_H


#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;

#define PI 3.141592
#define NUMOFSLAVES 12 // joint motor 갯수


double T = 0.0001;
double cutoff = 150;



double lowpassfilter(double input, double input_old, double output_old, double cutoff_freq) 
{
    double time_const = 1 / (2 * PI * cutoff_freq);
    double output = 0;

    output = (T * (input + input_old) - (T - 2 * time_const) * output_old) / (T + 2 * time_const);

    return output;
}


double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
{
    double time_const = 1 / (2 * PI * cutoff_freq);
    double output = 0;

    output = (2 * (input - input_old) - (0.0001 - 2 * time_const) * output_old) / (0.0001 + 2 * time_const);

    return output;
}
// #endif