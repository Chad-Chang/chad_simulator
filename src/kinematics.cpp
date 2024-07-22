#include "kinematics.h"

Kinematics::Kinematics() {
  jnt2bi << 1,0,
            1,1;
  bi2jnt << 1,0,
            -1,1;
}

// 다리별로 R과 theta의 값을 계산 => biarticular joint vel -> rotating vel
void Kinematics::Cal_RW(double thm, double thb, double thmdot, double thbdot, int Leg_num)
{

  double th2 = thb - thm;
  veljoint << thmdot, thbdot;

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
  
  velRW = RWJacobian*veljoint;  

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
Vector2d Kinematics::get_posRW_error(int idx) // idx = 0이면 현재 값, idx = 1이면 이전 값
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


Vector2d Kinematics ::get_velRW_error(int idx) // idx = 0이면 현재 값, idx = 1이면 이전 값
{

  Vector2d RWvel_error;
  Vector2d RWvel_error_old;

  if (idx == 0) {
    RWvel_error[0] = velRW_error[0][0];
    RWvel_error[1] = velRW_error[1][0];
    return RWvel_error;

  } else {
    RWvel_error_old[0] = velRW_error[0][1];
    RWvel_error_old[1] = velRW_error[1][1];
    return RWvel_error_old;
  }
}


void Kinematics::model_param_cal(double thm,double thb)
{
  double th2 = thb - thm;
  /* Trunk Parameters */
    double m_hip = 2.5;
    double m_trunk_front = 10.;
    double m_trunk_rear = 18.;
    double m_trunk = 4*m_hip + m_trunk_front + m_trunk_rear;

    /* Leg Parameters */
  
    double d_thigh = 0.11017;
    double d_shank = 0.12997;    // printf("d_thigh : %f, d_shank : %f \n", d_thigh, d_shank);

    double m_thigh = 1.017;
    double m_shank = 0.143;
    double m_leg = m_thigh + m_shank;
    double m_total = m_trunk + 4*m_leg;

    double Izz_thigh = 0.0057;
    double Izz_shank = 8.0318e-04; // MoI of shank w.r.t. CoM

    double Jzz_thigh =
        Izz_thigh + m_thigh * pow(d_thigh, 2); // MoI of thigh w.r.t. HFE
    double Jzz_shank =
        Izz_shank + m_shank * pow(d_shank, 2); // MoI of thigh w.r.t. KFE

    double M1 = Jzz_thigh + m_shank * pow(L, 2);
    double M2 = m_shank * d_shank * L * cos(th2);
    double M12 = Jzz_shank;

    Lamda_nominal_FOB(0,0) = M1;
    Lamda_nominal_FOB(0,1) = M12;
    Lamda_nominal_FOB(1,0) = M12;
    Lamda_nominal_FOB(1,1) = M2;

    double JzzR_thigh  = Jzz_thigh + Jzz_shank + m_shank * pow(L, 2) - 2 * m_shank * d_shank * L * cos(th2);
    double JzzR_couple = Jzz_thigh + m_shank * pow(L, 2) - Jzz_shank;
    double JzzR_shank = Jzz_thigh + Jzz_shank+ m_shank * pow(L, 2) + 2 * m_shank * d_shank * L * cos(th2);
    // printf("JzzR_thigh : %f, JzzR_shank : %f, JzzR_couple : %f \n", JzzR_thigh, JzzR_shank,
    // JzzR_couple);

    MatInertia_RW(0,0) = JzzR_thigh / (4 * pow(L, 2) * pow(sin(th2 / 2), 2));
    MatInertia_RW(0,1) = JzzR_couple / (2 * pow(L, 2) * sin(th2));
    MatInertia_RW(1,0) = JzzR_couple / (2 * pow(L, 2) * sin(th2));
    MatInertia_RW(1,1) = JzzR_shank / (4 * pow(L, 2) * pow(cos(th2 / 2), 2));
        
    Inertia_DOB(0,0) = MatInertia_RW(0,0);
    Inertia_DOB(0,1) = 0;
    Inertia_DOB(1,0) = 0;
    Inertia_DOB(1,1) = MatInertia_RW(1,1);
    
    //bi articular torque inertia 변환
    Lamda_nominal_DOB = RWJacobianTrans*Inertia_DOB*RWJacobian;
   
}

// trajectory 정의 및 error 값 계산 
void Kinematics::pos_trajectory(int traj_t, int Leg_num)
{
  double f_r = 0.01;
  //Leg_Num = FL(0), FR(1), RL(2), RR(3) 
  switch(Leg_num) // switch뮨 마지막에 break 았쓰면 제대로 작동 안함.
  {
    case 0: // FL position trajectory
    {
      ref_r_pos[0] = 0.1*sin(2*PI*f_r*0.001*traj_t) + L;
      ref_th_pos[0] = PI/2;
      posRW_error[0][0] = ref_r_pos[0] - posRW[0];
      posRW_error[1][0] = ref_th_pos[0] - posRW[1];
      r_pos_error[0] = posRW_error[0][0];
      th_pos_error[0] = posRW_error[1][0];
      break;
    }

    case 1: // FR position trajectory
    {
      ref_r_pos[1] = 0.1*sin(2*PI*f_r*0.001*traj_t) + L;
      ref_th_pos[1] = PI/2;
      posRW_error[0][0] = ref_r_pos[1] - posRW[0];
      posRW_error[1][0] = ref_th_pos[1] - posRW[1];
      
      r_pos_error[1] = posRW_error[0][0];
      th_pos_error[1] = posRW_error[1][0];
      break;
    }
    case 2: // RL position trajectory
    {
      ref_r_pos[2] = 0.1*sin(2*PI*f_r*0.001*traj_t) + L;
      ref_th_pos[2] = PI/2;
      posRW_error[0][0] = ref_r_pos[2] - posRW[0];
      posRW_error[1][0] = ref_th_pos[2] - posRW[1];
      r_pos_error[2] = posRW_error[0][0];
      th_pos_error[2] = posRW_error[1][0];
      break;
    }

    case 3: // RR position trajectory
    {
      ref_r_pos[3] = 0.1*sin(2*PI*f_r*0.001*traj_t) + L;
      ref_th_pos[3] = PI/2;
      posRW_error[0][0] = ref_r_pos[3] - posRW[0];
      posRW_error[1][0] = ref_th_pos[3] - posRW[1];
      r_pos_error[3] = posRW_error[0][0];
      th_pos_error[3] = posRW_error[1][0];
      break;
    }
  }


}

// trajectory 정의 및 error 값 계산 
void Kinematics::vel_trajectory(int traj_t, int Leg_num)
{
  double f_r = 0.01;
  //Leg_Num = FL(0), FR(1), RL(2), RR(3) 
  switch(Leg_num) // switch뮨 마지막에 break 았쓰면 제대로 작동 안함.
  {
    case 0: // FL vel trajectory
    {
      ref_r_vel[0] = 0.1*sin(2*PI*f_r*0.001*traj_t) + L;
      ref_th_vel[0] = 0;
      velRW_error[0][0] = ref_r_vel[0] - velRW[0];
      velRW_error[1][0] = ref_th_vel[0] - velRW[1];
      r_vel_error[0] = velRW_error[0][0];
      th_vel_error[0] = velRW_error[1][0];
      break;
    }

    case 1: // FR vel trajectory
    {
      ref_r_vel[1] = 0.1*sin(2*PI*f_r*0.001*traj_t) + L;
      ref_th_vel[1] = 0;
      velRW_error[0][0] = ref_r_vel[1] - velRW[0];
      velRW_error[1][0] = ref_th_vel[1] - velRW[1];
      
      r_vel_error[1] = velRW_error[0][0];
      th_vel_error[1] = velRW_error[1][0];
      break;
    }
    case 2: // RL vel trajectory
    {
      ref_r_vel[2] = 0.1*sin(2*PI*f_r*0.001*traj_t) + L;
      ref_th_vel[2] = 0;
      velRW_error[0][0] = ref_r_vel[2] - velRW[0];
      velRW_error[1][0] = ref_th_vel[2] - velRW[1];
      r_vel_error[2] = velRW_error[0][0];
      th_vel_error[2] = velRW_error[1][0];
      break;
    }

    case 3: // RR vel trajectory
    {
      ref_r_vel[3] = 0.1*sin(2*PI*f_r*0.001*traj_t) + L;
      ref_th_vel[3] = 0;
      velRW_error[0][0] = ref_r_vel[3] - velRW[0];
      velRW_error[1][0] = ref_th_vel[3] - velRW[1];
      r_vel_error[3] = velRW_error[0][0];
      th_vel_error[3] = velRW_error[1][0];
      break;
    }
  }


}

