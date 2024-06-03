#include "controller.h"

Controller::Controller()
{

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
//
double Controller::posPID(Vector2d posRW_err, Vector2d posRW_err_old, int idx,int Leg_num)
{
  kp = get_posPgain(Leg_num, idx);
  ki = get_posIgain(Leg_num, idx);
  kd = get_posDgain(Leg_num, idx);
  cutoff_freq = get_posD_cutoff(Leg_num, idx);
  
  double tau = 1 / (2 * PI * cutoff_freq);

  
  Pos_P_term[idx][0] = kp * posRW_err[idx];
  Pos_I_term[idx][0] = ki * T / 2 * (posRW_err[idx] + posRW_err_old[idx]) + Pos_I_term[idx][1];
  Pos_D_term[idx][0] = 2 * kd / (2 * tau + T) * (posRW_err[idx] - posRW_err_old[idx]) -
                       (T - 2 * tau) / (2 * tau + T) * Pos_D_term[idx][1]; // 이 함수 내에서 r_err는 주소값 but [0]와 같은 배열 위치로 원소를
                                                                           // 특정해주면 그 부분의 value가 된다.(이건 그냥 c++ 문법)
  posPID_output[idx] = Pos_P_term[idx][0] + Pos_D_term[idx][0] + Pos_I_term[idx][0];
  
  setDelayData();
  return posPID_output[idx];
  //
  
}

