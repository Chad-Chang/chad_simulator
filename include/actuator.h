#ifndef ACTUATOR_H
#define ACTUATOR_H
#include <Eigen/Core>
#include <Eigen/Dense>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#define PI 3.141592
#define NUMOFSLAVES 12 // joint motor 갯수

using namespace Eigen;


#define Torque_constant 0.035
#define Gear_ratio 100
class Actuator
{
    private:
        double Motor_initial_pos; // 모터 초기 자세
        double Motor_pos_offset; // 모터 offset

        // ////////////// DATA //////////////////////
        double Motor_pos; 
        double Motor_vel;
        double Motor_torque;

        double Motor_taget_torque;
        
        int Motor_Num;

        double Pterm[NUMOFSLAVES][3]{0,};
        double Iterm[NUMOFSLAVES][3]{0,};
        double Dterm[NUMOFSLAVES][3]{0,};
    public:
        Actuator(int Motor_num, double motor_init_pos);
        void DATA_reset();
        void DATA_Receive(const mjModel* m, mjData* d);
        void DATA_Send(const mjModel* m,mjData* d);
        void DATA_unit_change();

        // private 변수 불러오기
        double getMotor_pos() { return Motor_pos; }; // return first address of Motor_pos
        double getMotor_vel() { return Motor_vel; };
        double getMotor_torque() { return Motor_torque; };
};


#endif // ACTUATOR_H
