#ifndef ACTUATOR_H
#define ACTUATOR_H

// #include<cstdint>
// #include <cmath>
#include <stdio.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <iostream>

#define NUMOFSLAVES 12

using namespace std;

class Actuator
{
    private:
        double PI = 3.141592;
        double target_torque{0};
        double target_speed{0};
        double target_position{0};

        // Initial position //
        double Motor_initial_pos;
        double Motor_pos_offset;
        // ////////////// DATA //////////////////////
        double Motor_pos[2];
        double Motor_vel[2];
        double Motor_acc[2];
        double Motor_torque;
        // ///////////// Which motor ///////////////
        int Motor_Num;


        // /////////// Caculated ///////////////////

        double Pterm[NUMOFSLAVES][3]{0,};
        double Iterm[NUMOFSLAVES][3]{0,};
        double Dterm[NUMOFSLAVES][3]{0,};

    public:
        // constructor
        Actuator(int Motor_num, double motor_init_pos);
        // actuator data reset
        void DATA_reset();
        // encoder data read
        void DATA_Receive(mjData* d);
        // torque input send
        void DATA_Send(mjData* d,double torque);
        void setDelayData();
        void acc_cal(double cutoff);

        // private 변수 return
        double getMotor_pos() { return Motor_pos[0]; }; // return first address of Motor_pos
        double getMotor_vel() { return Motor_vel[0]; };
        double getMotor_acc() { return Motor_acc[0]; };
        double getMotor_torque() { return Motor_torque; };
};

#endif // ACTUATOR_H
