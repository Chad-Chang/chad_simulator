#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "j_controller.h"
using namespace Eigen;
using namespace std;


class RW_Controller : public J_Controller
{
  double cutoff_freq = 150; 
  double PI = 3.141592;
  double T = 0.001;

  // PID //
  Vector4d RW_r_posPgain = {40,40,40,40}; // FL FR RL RR leg
  Vector4d RW_r_posIgain;
  Vector4d RW_r_posDgain;
  Vector4d RW_r_posD_cutoff = {cutoff_freq,cutoff_freq,cutoff_freq,cutoff_freq};

  Vector4d RW_th_posPgain = {40,40,40,40}; // FL FR RL RR
  Vector4d RW_th_posIgain;
  Vector4d RW_th_posDgain;
  Vector4d RW_th_posD_cutoff= {cutoff_freq,cutoff_freq,cutoff_freq,cutoff_freq};

  Vector2d RW_posPID_output;
  
  
//   // Using in Function
  double RW_Pos_P_term[2][2]; // first column is about r, second column is about theta
  double RW_Pos_I_term[2][2];
  double RW_Pos_D_term[2][2];
  double RW_kp;
  double RW_ki;
  double RW_kd;


public:
  RW_Controller();
//***************************************** RW 제어기  
  void rw_setDelayData();

  // r방향 PID control : r0, theta 방향 PID control : th1 각각 따로
  double rw_posPID(Vector2d posRW_err, Vector2d posRW_err_old, int r0th1, int Leg_num); // idx:  r(=0), th(=1)중 어떤 state의 PD control?
//   Vector2d velPID();                                                                 // Leg_num: FL-0 FR-1 RL-2 RR-3
  void rw_set_gain(Vector4d r_Pgain,Vector4d r_Dgain,Vector4d r_Igain,Vector4d th_Pgain,Vector4d th_Dgain,Vector4d th_Igain);
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


};

#endif // CONTROLLER_H
