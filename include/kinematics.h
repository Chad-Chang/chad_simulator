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
        Vector2d posRW; // r, th 순서의 2x1벡터
        Vector2d velRW; // r, th 순서의 2x1벡터
        
        double posRW_error[2][2];
        double velRW_error[2][2];
        //******************
        double r_pos_error[4];// 
        double th_pos_error[4];
        
        double r_posRW[4];
        double th_posRW[4];

        double ref_r_pos[4];
        double ref_th_pos[4];

        ///////////동역학파라미터////////////
        double m_hip = 2.5;
        double m_trunk_front = 10.;
        double m_trunk_rear = 18.;
        double m_trunk = 4*m_hip + m_trunk_front + m_trunk_rear;
        double m_thigh = 1.017;
        double m_shank = 0.143;
        double m_leg = m_thigh + m_shank;
        double m_total = m_trunk + 4*m_leg;

        double d_thigh = 0.11017;
        double d_shank = 0.12997;

        double Izz_thigh = 0.0057;
        double Izz_shank = 8.0318e-04;
        //******************
        Matrix2d Jacobian;
        Matrix2d JacobianTrans;
        Matrix2d Jacobian_inv;
        Matrix2d JacobianTrans_inv;
        Matrix2d RW_Matrix;
        Matrix2d A;
        Matrix2d Lambda;

  public:
    Kinematics();
    
    // rw error를 업데이트 
    void set_DelayDATA();

    // trajectory와 error값을 계산해줌.
    void pos_trajectory(int traj_t, int Leg_num);
    
    // 다리 FK -> (biarticular 기준) 현재 다리의 각도를 이용해 rw각도로 변환
    void Cal_RW(double thm, double thb, int Leg_num); // thm = th1, thb = th1+th2
    void Cal_RW_inertia(double thm, double thb); // thm = th1, thb = th1+th2
    
    //rw jacobian
    Matrix2d RW_Jacobian(){ return Jacobian; };

    // private 변수 출력 
    Vector2d get_posRW() { return posRW; };
    Vector2d get_posRW_error(int idx);
    Matrix2d get_RW_Jacobian() { return Jacobian; };
    Matrix2d get_RW_Jacobian_Trans() { return JacobianTrans; };
    

    
    
};