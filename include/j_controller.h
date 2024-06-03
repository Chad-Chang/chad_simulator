#ifndef J_CONTROLLER_H_
#define J_CONTROLLER_H_

#include <math.h>
#include <stdio.h>
#define pi 3.141592

class J_Controller
{
    public:
        J_Controller();
        double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq);

        // gain setting
        void j_set_gain(double Kp, double Ki, double Kd){j_Kp_ = Kp; j_Ki_ = Ki; j_Kd_ = Kd; }
        // constrain
        double j_constraint(double v, double v_min, double v_max){return (v> v_max)?v_max : (v<v_min)?v_min:v;}
        
        // PID operator
        double j_posPID(double target, double current_ang, double dt, double cutoff);
        
        // old 값들 update
        void j_setDelayData(){j_D_term_[1] = j_D_term_[0]; j_I_term_[1] = j_I_term_[0];}

    private:
        double j_Kp_, j_Kd_, j_Ki_; 
        double j_I_term_[2] = {0};
        double j_D_term_[2] = {0};
        double j_P_term_ = 0;
        double j_imax_;
        double j_err[2] = {0};

};
#endif // TRAJECTORY_H_