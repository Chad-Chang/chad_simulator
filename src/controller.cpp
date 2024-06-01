#include "controller.h"
#define PI 3.141592
double T = 0.001;

Controller::Controller()
{

}

double Controller::posPID(Vector2d posRW_err, Vector2d posRW_err_old, int r0th1,int Leg_num)
                                       // posRW_err[0] : R direction
                                       // posRW_err[1] : th direction
                                       // posRW_err_old same
{
    kp = get_posPgain(Leg_num, r0th1); //
    ki = get_posIgain(Leg_num, r0th1); //
    kd = get_posDgain(Leg_num, r0th1); //
    cutoff_freq = get_posD_cutoff(Leg_num, r0th1);

    double tau = 1 / (2 * PI * cutoff_freq);

    // pos_N_term[r(0), th(1)][curr(0),old(1)]

    Pos_P_term[r0th1][0] = kp * posRW_err[r0th1];
    Pos_I_term[r0th1][0] = ki * T / 2 * (posRW_err[r0th1] + posRW_err_old[r0th1]) + Pos_I_term[r0th1][1];
    Pos_D_term[r0th1][0] = 2 * kd / (2 * tau + T) * (posRW_err[r0th1] - posRW_err_old[r0th1]) -
                        (T - 2 * tau) / (2 * tau + T) * Pos_D_term[r0th1][1]; // 이 함수 내에서 r_err는 주소값 but [0]와 같은 배열 위치로 원소를
                                                                            // 특정해주면 그 부분의 value가 된다.(이건 그냥 c++ 문법)
    posPID_output[r0th1] = Pos_P_term[r0th1][0] + Pos_D_term[r0th1][0] + Pos_I_term[r0th1][0];
    setDelayData();

    return posPID_output[r0th1];
  //
  
}


void Controller::setDelayData() {
  for (int i = 0; i < 2; i++) //[i][0] = z^0, [i][1] = z^1 ->  delay data 만들어 주는 function
  {
    /****************** Delay Data ******************/
    // r
    Pos_D_term[i + 1][0] = Pos_D_term[i][0];
    Pos_I_term[i + 1][0] = Pos_I_term[i][0];
    Pos_P_term[i + 1][0] = Pos_P_term[i][0];

    // th
    Pos_D_term[i + 1][1] = Pos_D_term[i][1];
    Pos_I_term[i + 1][1] = Pos_I_term[i][1];
    Pos_P_term[i + 1][1] = Pos_P_term[i][1];
  }
}