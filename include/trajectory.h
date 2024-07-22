#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;

class trajectory
{
private:

  int pos_traj_t = 0;
  int vel_traj_t = 0;
  
  // SLIP profile
  Matrix2d SLIP_profile;
   
  
  
  
public:
  trajectory();
  
  // test trajecotry code
  Vector2d test_rw_pos_cos_traj(MatrixXd rw_pos, MatrixXd rw_pos_curr, int time_interval); 
  Vector2d test_rw_vel_cos_traj(MatrixXd rw_vel, MatrixXd rw_vel_curr, int time_interval);
  
  
  Matrix2d SLIP_traj(); // velocity profile, position profile include 
   
  
};

#endif // TRAJECTORY_H
