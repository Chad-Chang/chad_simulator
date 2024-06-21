#include "kinematics.h"

Kinematics::Kinematics() {
  jnt2bi << 1,0,
            1,1;
  bi2jnt << 1,0,
            -1,1;
}

// 다리별로 R과 theta의 값을 계산 => biarticular joint vel -> rotating vel
void Kinematics::Cal_RW(double thm, double thb, int Leg_num)
{

  double th2 = thb - thm;

  // biarticular to rotating position space
  RWJacobian(0, 0) = sin(th2 / 2);
  RWJacobian(0, 1) = -sin(th2 / 2);
  RWJacobian(1, 0) = cos(th2 / 2);
  RWJacobian(1, 1) = cos(th2 / 2);


  RWJacobian = L * RWJacobian;
  RWJacobianTrans = RWJacobian.transpose();
  RWJacobian_inv = RWJacobian.inverse();
  RWJacobianTrans_inv = RWJacobianTrans.inverse();
  posRW[0] = 2 * L * cos((thb - thm) / 2);
  posRW[1] = 0.5 * (thm + thb);
  
  r_posRW[Leg_num] = posRW[0];
  th_posRW[Leg_num] = posRW[1];
}

Matrix2d Kinematics::Cal_RW_inertia(double thm, double thb) // Jr Lamda Jr
{
  // Jacobian
  double th2 = thb - thm;
  RWfrac_J1 = Izz_thigh + m_thigh * pow(d_thigh, 2) + Izz_shank + m_shank * pow(d_shank, 2) + m_shank * pow(L, 2) - 2 * m_shank * d_shank * L * cos(th2);
  RWfrac_J2 = Izz_thigh + m_thigh * pow(d_thigh, 2) + Izz_shank + m_shank * pow(d_shank, 2) + m_shank * pow(L, 2) + 2 * m_shank * d_shank * L * cos(th2);
  Lambda_RW_momi << RWfrac_J1/(4 * pow(L, 2) * pow(sin(th2 / 2), 2)) ,0 ,0 ,RWfrac_J2/(4 * pow(L, 2) * pow(cos(th2/ 2), 2));
  return Lambda_RW_momi;
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
  double f = 0.0;

  
  
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


