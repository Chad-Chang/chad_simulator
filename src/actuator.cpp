#include "actuator.h"
#include <iostream>
using namespace std;

Actuator::Actuator(int Motor_num, double motor_init_pos) {
  Motor_Num = Motor_num;
  Motor_pos = motor_init_pos;
  
  Motor_initial_pos = motor_init_pos;
}

void Actuator::DATA_reset() //
{
  for (int i = 0; i < NUMOFSLAVES; i++) //[i][0] = z^0, [i][1] = z^1.  2x1으로 1x1은 현재 값, 2x1은 이전 값인데 그값 초기화
  {
    Motor_pos = 0;
    Motor_vel = 0;
    Motor_torque = 0;
  }
}

void Actuator::DATA_Receive(mjData* d) //Motor_num은 class를 선언할 때 Ethercat 통신 순서대로 이미 정해줌
{ 
  if(Motor_Num == 0) {Motor_pos = d->qpos[16]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[16]);}
  else if(Motor_Num == 1) {Motor_pos = d->qpos[17]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[17]);}
  else if(Motor_Num == 2) {Motor_pos = d->qpos[18]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[18]);}
  else if(Motor_Num == 3) {Motor_pos = d->qpos[15]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[15]);}
  else if(Motor_Num == 4) {Motor_pos = d->qpos[14]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[14]);}
  else if(Motor_Num == 5) {Motor_pos = d->qpos[13]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[13]);}
  else if(Motor_Num == 6) {Motor_pos = d->qpos[7]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[7]);}
  else if(Motor_Num == 7) {Motor_pos = d->qpos[8]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[8]);}
  else if(Motor_Num == 8) {Motor_pos = d->qpos[9]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[9]);}
  else if(Motor_Num == 9) {Motor_pos = d->qpos[12]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[12]);}
  else if(Motor_Num == 10) {Motor_pos = d->qpos[11]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[11]);}
  else if(Motor_Num == 11) {Motor_pos = d->qpos[10]; }//printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[10]);}
  // spine
  // else if(Motor_Num == 12) {Motor_pos = d->qpos[17]; printf("motor pos = %f , qpose = %f\n", Motor_pos, d->qpos[17]);}
  

  // switch(Motor_Num)
  // { 
  //   // RR
  //   case 0://haa RR
  //   {
  //     Motor_pos = d->qpos[16];
  //     cout << "aaaaaaaa" <<endl;
      
      
  //   }
  //   case 1:
  //   {
  //     Motor_pos = d->qpos[17];
      
  //   }
  //   case 2:
  //   { Motor_pos = d->qpos[18];
      
  //     }

  //   // // RL 
  //   // case 3:{
  //   //   Motor_pos = d->qpos[15];
      
  //   //   }
  //   // case 4:{
  //   //   Motor_pos = d->qpos[14];
      
  //   //   }
      
  //   // case 5://haa RL
  //   //  { Motor_pos = d->qpos[13];
      
  //   // }

  //   // FL
  //   case 6:{//haa FL
  //     Motor_pos = d->qpos[7];
  //     }
  //   case 7:{
  //     Motor_pos = d->qpos[8];
     
  //     }
  //   case 8:{
  //     Motor_pos = d->qpos[9];
  //    }
    
  //   // // FR
  //   // case 9:{
  //   //   Motor_pos = d->qpos[12];
      
  //   //   }
  //   // case 10:{
  //   //   Motor_pos = d->qpos[11];
     
  //   //   }
  //   // case 11:{//haa FR
  //   //   Motor_pos = d->qpos[10];
  //   //  }
  //   // // spine
  //   // case 13:{
  //   //   Motor_pos = d->qpos[13];
  //   // }
  //   default:{break;}
  }
  // cout<<"Motor num =  " << Motor_Num << "qpos = " << Motor_pos <<endl;
  // cout << "real qpos "<< d->qpos[7] << "  "<< d->qpos[11] << "  " << d->qpos[13] << "  "<< d->qpos[16] << "  "<<endl;

// }

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
