#include "actuator.h"
#include <iostream>
#include <stdio.h>
using namespace std;


Actuator::Actuator(int Motor_num, double motor_init_pos) {
  Motor_Num = Motor_num;
  Motor_pos = motor_init_pos;
  Motor_initial_pos = motor_init_pos;
}


void Actuator::DATA_reset() // 이거는 로봇의 configuration에 따라 바꿔야 할 듯
{
  for (int i = 0; i < NUMOFSLAVES; i++) //[i][0] = z^0, [i][1] = z^1.  2x1으로 1x1은 현재 값, 2x1은 이전 값인데 그값 초기화
  {
    Motor_pos = 0;
    Motor_vel = 0;
    Motor_torque = 0;
  }
}


void Actuator::DATA_Send(const mjModel* m, mjData* d) //joint 토크입력 넣어주기
{
    d->ctrl[Motor_Num] = 100*Motor_taget_torque;
    // cout << "input " << Motor_Num<< ":"<< 100*Motor_taget_torque << endl;
}


void Actuator::DATA_Receive(const mjModel* m, mjData* d) //joint position, vel, acceleration 읽어오기
{  
    Motor_pos = d->qpos[Motor_Num+7];
    // cout << "hello " << Motor_Num<< ":"<< Motor_pos << endl;
}


