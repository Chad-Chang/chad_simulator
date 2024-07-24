#include "rw_controller.h"
#include <iostream>
using namespace std;
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


double RW_Controller::rw_posPID(Vector2d posRW_err, Vector2d posRW_err_old, int idx, int Leg_num) // idx:  r(=0), th(=1)중 어떤 state의 PD control?
{
  pos_kp = rw_get_posPgain(Leg_num, idx);
  pos_ki = rw_get_posIgain(Leg_num, idx);
  pos_kd = rw_get_posDgain(Leg_num, idx);
  pos_cutoff_freq = rw_get_posD_cutoff(Leg_num, idx);
  
  double tau = 1 / (2 * PI * pos_cutoff_freq);

  
  RW_Pos_P_term[idx][0] = pos_kp * posRW_err[idx];
  RW_Pos_I_term[idx][0] = pos_ki * T / 2 * (posRW_err[idx] + posRW_err_old[idx]) + RW_Pos_I_term[idx][1];
  RW_Pos_D_term[idx][0] = 2 * pos_kd / (2 * tau + T) * (posRW_err[idx] - posRW_err_old[idx]) -
                       (T - 2 * tau) / (2 * tau + T) * RW_Pos_D_term[idx][1]; // 이 함수 내에서 r_err는 주소값 but [0]와 같은 배열 위치로 원소를
                                                                           // 특정해주면 그 부분의 value가 된다.(이건 그냥 c++ 문법)
  RW_posPID_output[idx] = RW_Pos_P_term[idx][0] + RW_Pos_D_term[idx][0] + RW_Pos_I_term[idx][0];

  rw_setDelayData();
  return RW_posPID_output[idx];

}

double RW_Controller::rw_velPID(Vector2d velRW_err, Vector2d velRW_err_old, int idx, int Leg_num)
{
  vel_kp = rw_get_velPgain(Leg_num, idx);
  vel_ki = rw_get_velIgain(Leg_num, idx);
  vel_kd = rw_get_velDgain(Leg_num, idx);
  vel_cutoff_freq = rw_get_velD_cutoff(Leg_num, idx);
  
  double tau = 1 / (2 * PI * vel_cutoff_freq);

  
  RW_vel_P_term[idx][0] = vel_kp * velRW_err[idx];
  RW_vel_I_term[idx][0] = vel_ki * T / 2 * (velRW_err[idx] + velRW_err_old[idx]) + RW_vel_I_term[idx][1];
  RW_vel_D_term[idx][0] = 2 * vel_kd / (2 * tau + T) * (velRW_err[idx] - velRW_err_old[idx]) -
                       (T - 2 * tau) / (2 * tau + T) * RW_vel_D_term[idx][1]; // 이 함수 내에서 r_err는 주소값 but [0]와 같은 배열 위치로 원소를
                                                                           // 특정해주면 그 부분의 value가 된다.(이건 그냥 c++ 문법)
  RW_velPID_output[idx] = RW_vel_P_term[idx][0] + RW_vel_D_term[idx][0] + RW_vel_I_term[idx][0];

  rw_setDelayData();
  return RW_posPID_output[idx];

}

double RW_Controller::admittance(double omega_n, double zeta, double k) // position feedback
{
  // admittance control
  // admittance control은 현재 사용하지 않음
  // 현재 omega_n, zeta, k 로 tunning 하고 있는데, 변환식을 통해 아래에 적어주면 된다
    double ad_M = k/(pow(omega_n,2));
    double ad_B = 2*zeta*k/omega_n;
    double ad_K = k;

    double c1 = 4 * ad_M + 2 * ad_B * T + ad_K * pow(T, 2);
    double c2 = -8 * ad_M + 2 * ad_K * pow(T, 2);
    double c3 = 4 * ad_M - 2 * ad_B * T + ad_K * pow(T, 2);
   
   deltaPos[0] =
        (pow(T, 2) * forceExt_hat[0] + 2 * pow(T, 2) * forceExt_hat[1] +
            pow(T, 2) * forceExt_hat[2] - c2 * deltaPos[1] - c3 * deltaPos[2]) / c1;


  return deltaPos[0];
}

void RW_Controller::DOBinitial()
{ //Old값 초기화
//   for(int i = 0; i < 2; i++)
//    {
//      T_dob(i,1) = 0;
//      tauDist_hat(i,1) = 0;
//    }
    // 전체 초기화
    T_dob << 0,0,0,0;
    tauDist_hat << 0,0,0,0;
}

void RW_Controller::FOBinitial()
{ 
  T_fob << 0,0,0,0;
  tauExt_hat << 0,0,0,0;
  for(int i = 0 ; i< 3; i++)forceExt_hat[i] = 0;
}

void RW_Controller::init()
{
  DOBinitial();
  FOBinitial();
}


Vector2d RW_Controller::DOBRW(Vector2d DOB_output ,Matrix2d Lamda_nominal_DOB,Vector2d acc ,double cut_off ,int flag)

{
  //DOB_output이 한 step 이전 값이다. 그래서 여기 안에서 setting 안해줘도됨
  // old 값 initial 0으로 해줘야함
  // UI에 넣어야할 내용은 cut_off, flag
  double time_const = 1 / (2 * M_PI * cut_off); 
 // 정의는 여기서
  
    
    T_dob.col(1) = T_dob.col(0);
    // tauDist_hat(i,1) = tauDist_hat(i,0);
    tauDist_hat.col(1) = tauDist_hat.col(0);

  
  
  lhs_dob = DOB_output; // desired - DOBoutput

  rhs_dob = Lamda_nominal_DOB * acc;
  // calculate dob output
  
  T_dob.col(0) = lhs_dob - rhs_dob;  
  
  // cout << T_dob.col(0) << endl;
  
  if(flag == true)
  {
    // for(int i = 0 ; i <2 ; i++)
    // {
    //   tauDist_hat(i,0) = (2*(T_dob(i,0)+T_dob(i,1))-(T-2*time_const)*(tauDist_hat(i,1))/(T+2*time_const));
    // }
    tauDist_hat.col(0) = (2*(T_dob.col(0)+T_dob.col(1))-(T-2*time_const)*(tauDist_hat.col(1))/(T+2*time_const));
    
  }
  // else // no DOB 
  // {
  //   for(int i = 0 ; i <2 ; i++)
  //   {
  //     tauDist_hat(i,0) = 0 ;
  //   }
  // }
  

  
  Vector2d result;
  
  result[0] = tauDist_hat(0,0);
  result[1] = tauDist_hat(1,0);
  
  cout<< "DOB = " << result[0] << " " << result[1] << endl;
  return result;
  
}


void RW_Controller::FOBRW(Vector2d DOB_output,Matrix2d Lamda_FOB,Matrix2d Jacobian_T_inv,Vector2d acc ,double cut_off ,int flag)
{
  //DOB_output이 한 step 이전 값이다. 그래서 여기 안에서 setting 안해줘도됨
  // old 값 initial 0으로 해줘야함
  // UI에 넣어야할 내용은 cut_off, flag
  // Jacobian도 가져와야함
  double time_const = 1 / (2 * M_PI * cut_off);
  Vector2d result;
  
  
  lhs_dob = DOB_output; // desired - DOBoutput
  rhs_dob = Lamda_FOB * acc;
  
  // calculate dob output
  for(int i=  0; i <2 ; i++) // 
  {
    T_fob(i,0) = lhs_dob[i] - rhs_dob[i];
  }
  
  if(flag == true)
  {
    for(int i = 0 ; i< 2 ; i++)
      tauExt_hat(i,0) = (2* (T_fob(i,0) + T_fob(i,1)) - (T - 2*time_const)*tauExt_hat(i,1))/(T+2*time_const);
  }
  else
  {
    for(int i = 0 ; i <2 ; i++)
      tauExt_hat(i,0) = 0 ;
  }
  
  result = Jacobian_T_inv * tauExt_hat.col(0); // 0th colum is current tauExt_hat
  
  //old값 update
  T_fob.col(1) = T_fob.col(0);
  tauExt_hat.col(1) = tauExt_hat.col(0) ;
  
  // buble arrangement before 
  forceExt_hat[2] = forceExt_hat[1];
  forceExt_hat[1] = forceExt_hat[0];
  
  forceExt_hat[0] = result[0]; // r direction
  // cout << forceExt_hat[0]<< endl ;
}