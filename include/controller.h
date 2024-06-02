// #pragma once
#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;



// 다리별로 컨트롤러를 만들어줌 (FL FR RL RR leg)
class Controller 
{
    // FL FR RL RR
    private:
        double cutoff_freq = 150;
        // R방향 gain
        Vector4d RW_r_posPgain = {40, 40, 40, 40}; // 다리별로
        Vector4d RW_r_posIgain = {0,0,0,0}; // 다리별로
        Vector4d RW_r_posDgain= {0,0,0,0}; // 다리별로
        Vector4d RW_r_posD_cutoff = {cutoff_freq,cutoff_freq,cutoff_freq,cutoff_freq}; // 다리별로

        // theta방향 gain
        Vector4d RW_th_posPgain = {40, 40, 40, 40}; // 다리별로; 
        Vector4d RW_th_posIgain = {0,0,0,0};
        Vector4d RW_th_posDgain = {0,0,0,0};
        Vector4d RW_th_posD_cutoff = {cutoff_freq,cutoff_freq,cutoff_freq,cutoff_freq};

        Vector2d posPID_output;
        

        // position gain(2x2 matrix)
        // 1행은 r방향, 2행은 thetat방향
        double Pos_P_term[2][2]; 
        double Pos_I_term[2][2];
        double Pos_D_term[2][2];
        double kp=40;
        double ki;
        double kd;


    public:
        Controller();
        
        // Data set//private 변수 call하기 
        
        // idx:  r(=0), th(=1)중 어떤 state의 PD control?
        double posPID(Vector2d posRW_err, Vector2d posRW_err_old, int r0th1, int Leg_num); 
        Vector2d velPID();  // Leg_num: FL-0 FR-1 RL-2 RR-3
        void setDelayData();

        // ******************* private 변수 call하기 *************************//
        double get_posPgain(int Leg_num, int r0th1) {
            if (r0th1 == 0)
                {
                    printf("pgaint_r = %f\n",RW_r_posPgain[Leg_num]);
                    return RW_r_posPgain[Leg_num];
                    
                }
            else
            {
                printf("pgaint_th = %f\n",RW_th_posPgain[Leg_num]);
                return RW_th_posPgain[Leg_num];
            }
        };

        double get_posIgain(int Leg_num, int r0th1) {
            if (r0th1 == 0)
                return RW_r_posIgain[Leg_num];
            else
                return RW_th_posIgain[Leg_num];
        };

        double get_posDgain(int Leg_num, int r0th1) {
            if (r0th1 == 0)
                return RW_r_posDgain[Leg_num];
            else
                return RW_th_posDgain[Leg_num];
        };
        double get_posD_cutoff(int Leg_num, int r0th1) {
            if (r0th1 == 0)
                return RW_r_posD_cutoff[Leg_num];
            else
                return RW_th_posD_cutoff[Leg_num];
        };
        // ********************************************//
};
#endif // ACTUATOR_H
