
#include <math.h>
#include <stdio.h>
#include "j_controller.h"
#define pi 3.141592


// constructor 
J_Controller::J_Controller(){}

double J_Controller :: tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
{
    double time_const = 1 / (2 * pi * cutoff_freq);
    double output = 0;

    output = (2 * (input - input_old) - (0.0001 - 2 * time_const) * output_old) / (0.0001 + 2 * time_const);

    return output;
}

// PID operator
double J_Controller ::j_posPID(double target, double current_ang, double dt, double cutoff)
{   
    
    double t_const= 1/(2*pi*cutoff); // time constant
    j_err[0] = target-current_ang;
    j_P_term_ = j_Kp_*j_err[0];
    j_D_term_[0] = (j_Kd_*2*(j_err[0]-j_err[1])-(dt-2*t_const)*j_D_term_[1])/(2*t_const+dt);
    j_I_term_[0] = j_I_term_[1] + j_Ki_*dt/2*(j_err[0] + j_err[1]);
    j_I_term_[0] = j_constraint(j_I_term_[0],-10000,10000);
    // printf("PID = %f, %f, %f\n", P_term_, I_term_[0] , D_term_[0]);
    j_err[1] = j_err[0];
    return j_P_term_+ j_I_term_[0] + j_D_term_[0];
}

