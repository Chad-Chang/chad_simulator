#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "j_controller.h"
using namespace Eigen;
using namespace std;

extern double a;
// joint controller와 함수 형태나 형식 유사하게 만들기
class RW_Controller : public J_Controller
{
  private:
    float pos_cutoff_freq = 150.; 
    float vel_cutoff_freq = 150.; 
    float PI = 3.141592;
    float T = 0.001;
    // DOB
    Vector2d rhs_dob; Vector2d lhs_dob;
    Matrix2d T_dob; // disturbance before Qfilter
    Matrix2d tauDist_hat; //disturbance after Qfilter 
    
    Vector2d rhs_fob; Vector2d lhs_fob;  
    Matrix2d T_fob; // temporal tau exter
    Matrix2d tauExt_hat; // matrix form 

    double forceExt_hat[3]; // old값 초기화 해줘야함

    // admittance
    double deltaPos[3];

    // pose PID //
    Vector4d RW_r_posPgain = {4000,4000,4000,4000}; // FL FR RL RR leg
    Vector4d RW_r_posIgain;
    Vector4d RW_r_posDgain = {20,20,20,20};
    Vector4d RW_r_posD_cutoff = {pos_cutoff_freq,pos_cutoff_freq,pos_cutoff_freq,pos_cutoff_freq};

    Vector4d RW_th_posPgain = {4000,4000,4000,4000}; // FL FR RL RR
    Vector4d RW_th_posIgain= {0,0,0,0};
    Vector4d RW_th_posDgain= {20,20,20,20};
    Vector4d RW_th_posD_cutoff= {pos_cutoff_freq,pos_cutoff_freq,pos_cutoff_freq,pos_cutoff_freq};

    Vector2d RW_posPID_output;

    // vel PID //
    Vector4d RW_r_velPgain = {40,40,40,40}; // FL FR RL RR leg
    Vector4d RW_r_velIgain;
    Vector4d RW_r_velDgain;
    Vector4d RW_r_velD_cutoff = {vel_cutoff_freq,vel_cutoff_freq,vel_cutoff_freq,vel_cutoff_freq};

    Vector4d RW_th_velPgain = {40,40,40,40}; // FL FR RL RR
    Vector4d RW_th_velIgain;
    Vector4d RW_th_velDgain;
    Vector4d RW_th_velD_cutoff= {vel_cutoff_freq,vel_cutoff_freq,vel_cutoff_freq,vel_cutoff_freq};

    Vector2d RW_velPID_output;
    
    
  //   // Using in Function
    double RW_Pos_P_term[2][2]; // first column is about r, second column is about theta
    double RW_Pos_I_term[2][2];
    double RW_Pos_D_term[2][2];
    double pos_kp;
    double pos_ki;
    double pos_kd;

    double RW_vel_P_term[2][2]; // first column is about r, second column is about theta
    double RW_vel_I_term[2][2];
    double RW_vel_D_term[2][2];
    double vel_kp;
    double vel_ki;
    double vel_kd;


  public:
    RW_Controller();
  //***************************************** RW 제어기  
    // rw error값 업데이트
    void rw_setDelayData();

    // PID 컨트롤러 출력값 => 입력 변수 조금 바꾸고 싶음.
    double rw_posPID(Vector2d posRW_err, Vector2d posRW_err_old, int idx, int Leg_num); // idx:  r(=0), th(=1)중 어떤 state의 PD control?
                                                                                           // Leg_num: FL-0 FR-1 RL-2 RR-3
    double rw_velPID(Vector2d velRW_err, Vector2d velRW_err_old, int idx, int Leg_num); // idx:  r(=0), th(=1)중 어떤 state의 PD control?
                                                                                          // Leg_num: FL-0 FR-1 RL-2 RR-3
    

    // //DOB
    Vector2d DOBRW(Vector2d DOB_output ,Matrix2d Lamda_nominal_DOB,Vector2d acc,double cut_off ,int flag);
    void DOBinitial();

    // //FOB 
    void FOBRW(Vector2d DOB_output,Matrix2d Lamda_nominal_FOB,Matrix2d JacobianTrans,Vector2d acc ,double cut_off ,int flag);//flag 대신 of/off     
    void FOBinitial();

    //admittance
    double admittance(double omega_n, double zeta, double k);

    void init();
    // rw gain 값들 setting :: 각 변수마다 4x1 벡터로 입력 
    void rw_set_gain(Vector4d r_Pgain,Vector4d r_Dgain,Vector4d r_Igain,Vector4d th_Pgain,Vector4d th_Dgain,Vector4d th_Igain);
    
    
    // private 변수 출력값
    double rw_get_posPgain(int Leg_num, int r0th1) {
      if (r0th1 == 0)
        return RW_r_posPgain[Leg_num];
      else
        return RW_th_posPgain[Leg_num];
    };

    double rw_get_posIgain(int Leg_num, int r0th1) {
      if (r0th1 == 0)
        return RW_r_posIgain[Leg_num];
      else
        return RW_th_posIgain[Leg_num];
    };

    double rw_get_posDgain(int Leg_num, int r0th1) {
      if (r0th1 == 0)
        return RW_r_posDgain[Leg_num];
      else
        return RW_th_posDgain[Leg_num];
    };
    double rw_get_posD_cutoff(int Leg_num, int r0th1) {
      if (r0th1 == 0)
        return RW_r_posD_cutoff[Leg_num];
      else
        return RW_th_posD_cutoff[Leg_num];
    };

    double rw_get_velPgain(int Leg_num, int r0th1) {
      if (r0th1 == 0)
        return RW_r_velPgain[Leg_num];
      else
        return RW_th_velPgain[Leg_num];
    };

    double rw_get_velIgain(int Leg_num, int r0th1) {
      if (r0th1 == 0)
        return RW_r_velIgain[Leg_num];
      else
        return RW_th_velIgain[Leg_num];
    };

    double rw_get_velDgain(int Leg_num, int r0th1) {
      if (r0th1 == 0)
        return RW_r_velDgain[Leg_num];
      else
        return RW_th_velDgain[Leg_num];
    };
    double rw_get_velD_cutoff(int Leg_num, int r0th1) {
      if (r0th1 == 0)
        return RW_r_velD_cutoff[Leg_num];
      else
        return RW_th_velD_cutoff[Leg_num];
    };


};

#endif // CONTROLLER_H
