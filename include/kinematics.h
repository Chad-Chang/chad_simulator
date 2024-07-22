#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace std;


class Kinematics
{   
    private:
        double PI = 3.141592;
        double L = 0.25;

        Vector2d veljoint; // biarticular의 joint velocity
        
        Vector2d posRW; // r, th 순서의 2x1벡터
        Vector2d velRW; // r, th 순서의 2x1벡터
        
        double posRW_error[2][2];
        double velRW_error[2][2];
        //******************
        double r_pos_error[4];// 
        double th_pos_error[4];
        double r_vel_error[4];// 
        double th_vel_error[4];
        
        double r_posRW[4];
        double th_posRW[4];
        double r_velRW[4];
        double th_velRW[4];

        double ref_r_pos[4];
        double ref_th_pos[4];
        double ref_r_vel[4];
        double ref_th_vel[4];

        ///////////동역학파라미터////////////

        double d_thigh = 0.11017;
        double d_shank = 0.12997;

        double Izz_thigh = 0.0057;
        double Izz_shank = 8.0318e-04;

        double RWfrac_J1;
        double RWfrac_J2;
        double RWfrac_Jm;
        //******************
        Matrix2d RWJacobian;
        Matrix2d RWJacobianTrans;
        Matrix2d RWJacobian_inv;
        Matrix2d RWJacobianTrans_inv;

        Matrix2d Lambda_RW_momi; // RW position inertia matrix

        Matrix2d MatInertia_bi;
        Matrix2d MatInertia_RW;
        Matrix2d Inertia_DOB;
        Matrix2d Lamda_nominal_FOB;
        Matrix2d Lamda_nominal_DOB;

        Matrix2d jnt2bi; 
        Matrix2d bi2jnt;

  public:
    Kinematics();

    // trajectory와 error값을 계산해줌.
    void pos_trajectory(int traj_t, int Leg_num);
    void vel_trajectory(int traj_t, int Leg_num);
    
    // 다리 FK -> (biarticular 기준) 현재 다리의 각도를 이용해 rw각도로 변환
    void Cal_RW(double thm, double thb, double thmdot, double thbdot, int Leg_num); // thm = th1, thb = th1+th2

    void model_param_cal(double thm,double thb);
    // rw error를 업데이트 
    void set_DelayDATA();


    // private 변수 출력 
    Vector2d get_posRW() { return posRW; };
    Vector2d get_posRW_error(int idx);
    Vector2d get_velRW() {return velRW; };
    Vector2d get_velRW_error(int idx);

    Matrix2d get_RW_Jacobian() { return RWJacobian; };
    Matrix2d get_RW_Jacobian_Trans() { return RWJacobianTrans; };
    Matrix2d get_RW_Jacobian_Trans_inv() { return RWJacobianTrans_inv; };
    Matrix2d get_Lamda_nominal_FOB() { return Lamda_nominal_FOB; };
    Matrix2d get_Lamda_nominal_DOB() { return Lamda_nominal_DOB; };
    // Matrix2d get_matrix_jnt2bi(){return };
    // Matrix2d get_matrix_bi2jnt(){return };

    
    
};