#ifndef ACTUATOR_H
#define ACTUATOR_H

// #include<cstdint>
// #include <cmath>
#include <stdio.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <iostream>

#define NUMOFSLAVES 12

// using namespace std;

class Actuator
{
    private:
        double target_torque{0};
        double target_speed{0};
        double target_position{0};


        // Initial position //
        double Motor_initial_pos;
        double Motor_pos_offset;

        // ////////////// DATA //////////////////////
        double Motor_pos;
        double Motor_vel;
        double Motor_torque;

        double Motor_position;
        double Motor_velocity;

        // ///////////// Which motor ///////////////
        int Motor_Num;


        // /////////// Caculated ///////////////////

        double Pterm[NUMOFSLAVES][3]{0,};
        double Iterm[NUMOFSLAVES][3]{0,};
        double Dterm[NUMOFSLAVES][3]{0,};

    public:
        Actuator(int Motor_num, double motor_init_pos);
        void DATA_reset();
        void DATA_Receive(mjData* d);
        void DATA_Send(mjData* d,double torque);
        

        double getMotor_pos() { return Motor_pos; }; // return first address of Motor_pos
        double getMotor_vel() { return Motor_vel; };
        double getMotor_torque() { return Motor_torque; };
};

#endif // ACTUATOR_H
