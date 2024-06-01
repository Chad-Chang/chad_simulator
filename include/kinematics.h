
#ifndef GLOBAL_FUNCTION_H
#define GLOBAL_FUNCTION_H
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;


class Kinematics
{
    private:
        
        double L = 0.25; // thigh, shank길이
        Vector2d posRW; // r, th 2x1벡터
        Vector2d velRW; // r, th 2x1벡터

        double posRW_error[2][2]; // (1행 : r방향 , 2행 : th방향), (1열 : 현재값, 2열 : 다음값)
        double velRW_error[2][2];
        //******************
        // 사실 다리 하나씩만 해서 이거 인덱스 없는게 맞음.
        double r_pos_error[4]; 
        double th_pos_error[4];

        double r_posRW[4];
        double th_posRW[4];

        double ref_r_pos[4];
        double ref_th_pos[4];
        //******************
        Matrix2d Jacobian;
        Matrix2d JacobianTrans;
    

    public:
        Kinematics();
        void set_DelayDATA();
        Matrix2d RW_Jacobian(){ return Jacobian; };
        Vector2d get_posRW() { return posRW; };
        Vector2d get_posRW_error(int idx);
        double *get_velRW(double thm_dot, double thb_dot);
        Matrix2d get_RW_Jacobian() { return Jacobian; };
        Matrix2d get_RW_Jacobian_Trans() { return JacobianTrans; };

        // delete
        //    void RL_pos_trajectory(int &traj_t, bool isPressed, int Leg_num);
        void pos_trajectory(int traj_t, int Leg_num);
        void Cal_RW(double thm, double thb, int Leg_num);
    

};

#endif