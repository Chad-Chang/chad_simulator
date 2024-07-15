#include "actuator.h"
#include <iostream>
using namespace std;

Actuator::Actuator(int Motor_num, double motor_init_pos) {
  Motor_Num = Motor_num;
  Motor_pos[0] = motor_init_pos;
  Motor_initial_pos = motor_init_pos;
}
void Actuator::setDelayData()
{
  Motor_pos[1] = Motor_pos[0];
  Motor_vel[1] = Motor_vel[0];
  Motor_acc[1] = Motor_acc[0];
}
void Actuator::acc_cal(double cutoff)
{
  double time_const = 1 / (2 * PI * cutoff);
  double Ts = 0.001;

  Motor_acc[0] = (2 * (Motor_vel[0] - Motor_vel[1]) - 
                      (Ts - 2 * time_const) * Motor_acc[1]) / (Ts + 2 * time_const);
}
void Actuator::DATA_reset() //
{
  for (int i = 0; i < NUMOFSLAVES; i++) //[i][0] = z^0, [i][1] = z^1.  2x1으로 1x1은 현재 값, 2x1은 이전 값인데 그값 초기화
  {
    Motor_pos[0] = 0;
    Motor_vel[0] = 0;
    Motor_torque = 0;
  }
}

void Actuator::DATA_Receive(mjData* d) //Motor_num은 class를 선언할 때 Ethercat 통신 순서대로 이미 정해줌
{ 
  if(Motor_Num == 0) {Motor_pos[0] = d->qpos[16]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[16]);}
  else if(Motor_Num == 1) {Motor_pos[0] = d->qpos[17]; Motor_vel[0] = d->qvel[17]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[17]);}
  else if(Motor_Num == 2) {Motor_pos[0] = d->qpos[18]; Motor_vel[0] = d->qvel[18]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[18]);}
  else if(Motor_Num == 3) {Motor_pos[0] = d->qpos[15]; Motor_vel[0] = d->qvel[15]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[15]);}
  else if(Motor_Num == 4) {Motor_pos[0] = d->qpos[14]; Motor_vel[0] = d->qvel[14]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[14]);}
  else if(Motor_Num == 5) {Motor_pos[0] = d->qpos[13]; Motor_vel[0] = d->qvel[13]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[13]);}
  else if(Motor_Num == 6) {Motor_pos[0] = d->qpos[7]; Motor_vel[0] = d->qvel[7]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[7]);}
  else if(Motor_Num == 7) {Motor_pos[0] = d->qpos[8]; Motor_vel[0] = d->qvel[8]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[8]);}
  else if(Motor_Num == 8) {Motor_pos[0] = d->qpos[9]; Motor_vel[0] = d->qvel[9]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[9]);}
  else if(Motor_Num == 9) {Motor_pos[0] = d->qpos[12]; Motor_vel[0] = d->qvel[12]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[12]);}
  else if(Motor_Num == 10) {Motor_pos[0] = d->qpos[11]; Motor_vel[0] = d->qvel[11]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[11]);}
  else if(Motor_Num == 11) {Motor_pos[0] = d->qpos[10]; Motor_vel[0] = d->qvel[10]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[10]);}
  // spine
  // else if(Motor_Num == 12) {Motor_pos = d->qpos[17]; printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[17]);}

  }


void Actuator::DATA_Send(mjData* d, double torque) //
{
  if(Motor_Num == 0) {d->ctrl[9] = torque;}
  else if(Motor_Num == 1) {d->ctrl[10] = torque;}
  else if(Motor_Num == 2) {d->ctrl[11] = torque;}
  else if(Motor_Num == 3) {d->ctrl[8] = torque;}
  else if(Motor_Num == 4) {d->ctrl[7] = torque;}
  else if(Motor_Num == 5) {d->ctrl[6] = torque;}
  else if(Motor_Num == 6) {d->ctrl[0] = torque;}
  else if(Motor_Num == 7) {d->ctrl[1] = torque;}
  else if(Motor_Num == 8) {d->ctrl[2] = torque;}
  else if(Motor_Num == 9) {d->ctrl[5] = torque;}
  else if(Motor_Num == 10) {d->ctrl[4] = torque;}
  else if(Motor_Num == 11) {d->ctrl[3] = torque;}
    // d->ctrl[Motor_Num] = torque;
    // cout <<"Motor_num = " <<  Motor_Num<< ": "<< torque<<endl;
}
