#ifndef PID_H_
#define PID_H_

#include <math.h>
#include <stdio.h>
#include "funcs.h"
#define pi 3.141592

class PID{
    public:
        double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
        {
            double time_const = 1 / (2 * pi * cutoff_freq);
            double output = 0;

            output = (2 * (input - input_old) - (0.0001 - 2 * time_const) * output_old) / (0.0001 + 2 * time_const);

            return output;
        }
        // constructor 
        PID(double kp=0, double kd=0,double ki=0)
        {
            Kp_= kp; Kd_ = kd ; Ki_ = ki;
        }

        // gain setting
        void set_gain(double Kp, double Ki, double Kd)
        {
            Kp_ = Kp; Ki_ = Ki; Kd_ = Kd; 
        }
        // constrain
        double constraint(double v, double v_min, double v_max)
        {
            return (v> v_max)?v_max : (v<v_min)?v_min:v;
        }
        // PID operator
        double compute_PID(double err, double err_old, double dt, double cutoff)
        {
            double t_const= 1/(2*pi*cutoff); // time constant

            P_term_ = Kp_*err;
            D_term_[0] = (Kd_*2*(err-err_old)-(dt-2*t_const)*D_term_[1])/(2*t_const+dt);
            I_term_[0] = I_term_[1] + Ki_*dt/2*(err + err_old);
            I_term_[0] = constraint(I_term_[0],-10000,10000);
            // printf("PID = %f, %f, %f\n", P_term_, I_term_[0] , D_term_[0]);
            return P_term_+ I_term_[0] + D_term_[0];
        }
        // old 값들 update
        void update_PID()
        {
            D_term_[1] = D_term_[0]; I_term_[1] = I_term_[0];
        }

    private:
        double Kp_, Kd_, Ki_; 
        double I_term_[2] = {0};
        double D_term_[2] = {0};
        double P_term_ = 0;
        double imax_;

};

#endif // TRAJECTORY_H_
