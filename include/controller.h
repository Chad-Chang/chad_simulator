// #ifndef CONTROLLER_H
// #define CONTROLLER_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
// #include "j_controller.h"
using namespace Eigen;
using namespace std;


class Controller//(J_Controller)
{
private:
  double PI = 3.141592;
  double T = 0.001;
  double cutoff_freq = 150; 
  // PID //
  Vector4d RW_r_posPgain = {40,40,40,40}; // FL FR RL RR leg
  Vector4d RW_r_posIgain;
  Vector4d RW_r_posDgain;
  Vector4d RW_r_posD_cutoff = {cutoff_freq,cutoff_freq,cutoff_freq,cutoff_freq};

  Vector4d RW_th_posPgain= {40,40,40,40}; // FL FR RL RR
  Vector4d RW_th_posIgain;
  Vector4d RW_th_posDgain;
  Vector4d RW_th_posD_cutoff= {cutoff_freq,cutoff_freq,cutoff_freq,cutoff_freq};

  Vector2d posPID_output;
  
  
//   // Using in Function
  double Pos_P_term[2][2]; // first column is about r, second column is about theta
  double Pos_I_term[2][2];
  double Pos_D_term[2][2];
  double kp;
  double ki;
  double kd;


public:
  Controller();
//***************************************** RW 제어기  
  void setDelayData();

  // r방향 PID control : r0, theta 방향 PID control : th1 각각 따로
  double posPID(Vector2d posRW_err, Vector2d posRW_err_old, int r0th1, int Leg_num); // idx:  r(=0), th(=1)중 어떤 state의 PD control?
//   Vector2d velPID();                                                                 // Leg_num: FL-0 FR-1 RL-2 RR-3

  double get_posPgain(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_posPgain[Leg_num];
    else
      return RW_th_posPgain[Leg_num];
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


};

// #endif // CONTROLLER_H
