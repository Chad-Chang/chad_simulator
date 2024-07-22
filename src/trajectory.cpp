#include "trajectory.h"

trajectory::trajectory()
{}

Vector2d trajectory::test_rw_pos_cos_traj(MatrixXd rw_pos, MatrixXd rw_pos_curr, int time_interval)
{
  pos_traj_t ++; 
  Vector2d rw_pos_trajectory;
  rw_pos_trajectory << 0,0;
  
  if(pos_traj_t*0.001 <=time_interval)
  {
    for(int i = 0 ; i< 4; i++)
    {
      rw_pos_trajectory.col(i) = 0.5*(rw_pos.col(i)-rw_pos_curr.col(i))*(1-cos((M_PI/time_interval)*pos_traj_t*0.001))+rw_pos_curr.col(i);
    }
  }
  else
  {
    cout << " ===================== pos trajectory end ========================="<< endl;
    rw_pos_trajectory =  rw_pos_curr;
    pos_traj_t = 0;
  }
  return rw_pos_trajectory;
} 


Vector2d trajectory::test_rw_vel_cos_traj(MatrixXd rw_vel, MatrixXd r_vel_curr, int time_interval)
{
  vel_traj_t ++; 
  Vector2d rw_vel_trajectory(2,4);
  rw_vel_trajectory << 0,0,0,0,
                       0,0,0,0;
  
  if(vel_traj_t*0.001 <=time_interval)
  {
    for(int i = 0 ; i< 2; i++)
    {
      rw_vel_trajectory.col(i) = 0.5*(rw_vel.col(i)-r_vel_curr.col(i))*(1-cos((M_PI/time_interval)*vel_traj_t*0.001))+r_vel_curr.col(i);
    }
  }
  else
  {
    cout << " ===================== vel trajectory end ========================="<< endl;
    rw_vel_trajectory <<0,0; // stop
    vel_traj_t = 0;
  }
  return rw_vel_trajectory;
}

Matrix2d trajectory::SLIP_traj() // velocity profile, position profile include
{
  return SLIP_profile;
}
