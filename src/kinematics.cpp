#include "kinematics.h"

Kinematics::Kinematics() {

}

// 다리별로 R과 theta의 값을 계산
void Kinematics::Cal_RW(double thm, double thb, int Leg_num)
{

  double th2 = thb - thm;
  
  Jacobian(0, 0) = sin(th2 / 2);
  Jacobian(0, 1) = -sin(th2 / 2);
  Jacobian(1, 0) = cos(th2 / 2);
  Jacobian(1, 1) = cos(th2 / 2);

  Jacobian = L * Jacobian;
  JacobianTrans = Jacobian.transpose();
  
  posRW[0] = 2 * L * cos((thb - thm) / 2);
  posRW[1] = 0.5 * (thm + thb);
  
  r_posRW[Leg_num] = posRW[0];
  th_posRW[Leg_num] = posRW[1];

}

// 이전 값들 업데이트
void Kinematics::set_DelayDATA() {
  for (int i = 0; i < 2; i++) //[i][0] = z^0, [i][1] = z^1 ...
  {
    // Delay data
    posRW_error[i][1] = posRW_error[i][0];
    velRW_error[i][1] = velRW_error[i][0];
  }
}

// pose error값 출력
Vector2d Kinematics ::get_posRW_error(int idx) // idx = 0이면 현재 값, idx = 1이면 이전 값
{

  Vector2d RWpos_error;
  Vector2d RWpos_error_old;

  if (idx == 0) {
    RWpos_error[0] = posRW_error[0][0];
    RWpos_error[1] = posRW_error[1][0];
  
    return RWpos_error;
  } else {
    RWpos_error_old[0] = posRW_error[0][1];
    RWpos_error_old[1] = posRW_error[1][1];

    return RWpos_error_old;
  }
}




// trajectory 정의 및 error 값 계산 
void Kinematics::pos_trajectory(int traj_t, int Leg_num)
{
  
  // r direction
  // double f1 = 1;
  // double f2 = 0.05;
  // double f3 = 0.07;
  double f = 0.1;

  
  
  //Leg_Num = FL(0), FR(1), RL(2), RR(3) 
  switch(Leg_num) // switch뮨 마지막에 break 았쓰면 제대로 작동 안함.
  {
    case 0: // FL position trajectory
    {
      ref_r_pos[0] = 0.1*sin(2*PI*f*0.001*traj_t) + L;
      ref_th_pos[0] = PI/2;
      posRW_error[0][0] = ref_r_pos[0] - posRW[0];
      posRW_error[1][0] = ref_th_pos[0] - posRW[1];
      r_pos_error[0] = posRW_error[0][0];
      th_pos_error[0] = posRW_error[1][0];
      break;
    }

    case 1: // FR position trajectory
    {
      ref_r_pos[1] = 0.1*sin(2*PI*f*0.001*traj_t) + L;
      ref_th_pos[1] = PI/2;
      posRW_error[0][0] = ref_r_pos[1] - posRW[0];
      posRW_error[1][0] = ref_th_pos[1] - posRW[1];
      
      r_pos_error[1] = posRW_error[0][0];
      th_pos_error[1] = posRW_error[1][0];
      break;
    }
    case 2: // RL position trajectory
    {
      ref_r_pos[2] = 0.1*sin(2*PI*f*0.001*traj_t) + L;
      ref_th_pos[2] = PI/2;
      posRW_error[0][0] = ref_r_pos[2] - posRW[0];
      posRW_error[1][0] = ref_th_pos[2] - posRW[1];
      r_pos_error[2] = posRW_error[0][0];
      th_pos_error[2] = posRW_error[1][0];
      break;
    }

    case 3: // RR position trajectory
    {
      ref_r_pos[3] = 0.1*sin(2*PI*f*0.001*traj_t) + L;
      ref_th_pos[3] = PI/2;
      posRW_error[0][0] = ref_r_pos[3] - posRW[0];
      posRW_error[1][0] = ref_th_pos[3] - posRW[1];
      r_pos_error[3] = posRW_error[0][0];
      th_pos_error[3] = posRW_error[1][0];
      break;
    }
  }


}


