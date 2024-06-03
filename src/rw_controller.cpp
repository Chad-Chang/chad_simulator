#include "rw_controller.h"

RW_Controller::RW_Controller()
{

}

void RW_Controller :: rw_set_gain(Vector4d r_Pgain,Vector4d r_Igain,Vector4d r_Dgain,Vector4d th_Pgain,Vector4d th_Igain,Vector4d th_Dgain)
{
  RW_r_posPgain = r_Pgain;
  RW_r_posIgain = r_Igain;
  RW_r_posDgain = r_Dgain;
  RW_th_posPgain = th_Pgain;
  RW_th_posIgain = th_Igain;
  RW_th_posDgain = th_Dgain;
}


void RW_Controller::rw_setDelayData() {
  for (int i = 0; i < 2; i++) //[i][0] = z^0, [i][1] = z^1 ->  delay data 만들어 주는 function
  {
    /****************** Delay Data ******************/
    // r
    RW_Pos_D_term[i + 1][0] = RW_Pos_D_term[i][0];
    RW_Pos_I_term[i + 1][0] = RW_Pos_I_term[i][0];
    RW_Pos_P_term[i + 1][0] = RW_Pos_P_term[i][0];

    // th
    RW_Pos_D_term[i + 1][1] = RW_Pos_D_term[i][1];
    RW_Pos_I_term[i + 1][1] = RW_Pos_I_term[i][1];
    RW_Pos_P_term[i + 1][1] = RW_Pos_P_term[i][1];
  }
}


double RW_Controller::rw_posPID(Vector2d posRW_err, Vector2d posRW_err_old, int idx,int Leg_num)
{
  RW_kp = rw_get_posPgain(Leg_num, idx);
  RW_ki = rw_get_posIgain(Leg_num, idx);
  RW_kd = rw_get_posDgain(Leg_num, idx);
  cutoff_freq = rw_get_posD_cutoff(Leg_num, idx);
  
  double tau = 1 / (2 * PI * cutoff_freq);

  
  RW_Pos_P_term[idx][0] = RW_kp * posRW_err[idx];
  RW_Pos_I_term[idx][0] = RW_ki * T / 2 * (posRW_err[idx] + posRW_err_old[idx]) + RW_Pos_I_term[idx][1];
  RW_Pos_D_term[idx][0] = 2 * RW_kd / (2 * tau + T) * (posRW_err[idx] - posRW_err_old[idx]) -
                       (T - 2 * tau) / (2 * tau + T) * RW_Pos_D_term[idx][1]; // 이 함수 내에서 r_err는 주소값 but [0]와 같은 배열 위치로 원소를
                                                                           // 특정해주면 그 부분의 value가 된다.(이건 그냥 c++ 문법)
  RW_posPID_output[idx] = RW_Pos_P_term[idx][0] + RW_Pos_D_term[idx][0] + RW_Pos_I_term[idx][0];
  
  
  rw_setDelayData();
  return RW_posPID_output[idx];
}

