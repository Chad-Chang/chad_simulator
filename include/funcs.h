#pragma once
#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>


#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include "stdlib.h"
#include "string.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;

// #define G 9.81
double PI = 3.141592;
double T = 0.001;

char filename[] = "scene.xml";
char datafile[] = "data/DOB.csv";

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// Simulation End Time
double simend = 10;
// const int nv = 2; // DoF of system 

// Data Writing
FILE* fid;
int loop_index = 0;
const int data_frequency = 4; // frequency at which data is written to a file

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
double Ts = 0.001;
double cutoff = 150;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}
// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}
// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}
// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}


// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

//****************************
//This function is called once and is used to get the headers
void init_save_data() // csv파일의 데이터 명을 지정하는 함수 -> 한번만 실행
{
   
}

////***************************
////This function is called at a set frequency, put data here
void save_data(const mjModel* m, mjData* d) 
{   
    
}

double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
{
    double time_const = 1 / (2 * PI * cutoff_freq);
    double output = 0;

    output = (2 * (input - input_old) - (0.0001 - 2 * time_const) * output_old) / (0.0001 + 2 * time_const);

    return output;
}

double t_k =0 ;
double t_kold= 0 ;
int contol_loop = 4; // 0.0001ms초를 보장해주는 놈
void loop_tcheck()// loop time check해주는 놈
{ 
    t_k = d->time;
    printf("%f \n", t_k-t_kold);
    t_kold = t_k;
}

// 1 / (tau*s+1)
double lowpassfilter(double input, double input_old, double output_old, double cutoff_freq) 
{
    double time_const = 1 / (2 * PI * cutoff_freq);
    double output = 0;

    output = (Ts * (input + input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}

// /* Model Variables */
// struct ParamModel_
// {
//     /* Trunk Parameter */
//     double m_hip;         // mass of hip torso
//     double m_trunk_front; // mass of front trunk
//     double m_trunk_rear;  // mass of rear trunk
//     double m_trunk;       // total mass of trunk
//     double m_total;       // total robot mass

//     double Ixx_trunk;
//     double Iyy_trunk;
//     double Izz_trunk;

//     /* Leg Parameter */
//     double L; // leg length : thigh and shank links' length are assumed to be the same

//     double m_thigh; // mass of thigh link
//     double m_shank; // mass of shank link
//     double m_leg;   // mass of leg

//     double d_thigh; // CoM pos of thigh w.r.t HFE
//     double d_shank; // CoM pos of shank w.r.t KFE

//     double Izz_thigh; // MoI(z) of thigh w.r.t its CoM
//     double Izz_shank; // MoI(z) of shank w.r.t its CoM

//     double Jzz_thigh; // MoI(z) of thigh w.r.t HFE
//     double Jzz_shank; // MoI(z) of shank w.r.t KFE

//     double JzzR_thigh;
//     double JzzR_shank;
//     double JzzR_couple;

//     double MatInertia_bi[NDOF_LEG * NDOF_LEG];
//     double MatInertia_RW[NDOF_LEG * NDOF_LEG];
// };

// void model_param_cal(const mjModel* m, mjData* d, ParamModel_* param_model, StateModel_* state_model)
// {
//     /* Trunk Parameters */
//     param_model->m_hip = 2.5;
//     param_model->m_trunk_front = 10.;
//     param_model->m_trunk_rear = 18.;
//     param_model->m_trunk = 4 * param_model->m_hip + param_model->m_trunk_front + param_model->m_trunk_rear;

//     /* Leg Parameters */
//     param_model -> L = 0.25;
//     param_model -> d_thigh = 0.11017; // local position of CoM of thigh
//     param_model -> d_shank = 0.12997; // local position of CoM of shank

//     param_model->m_thigh = 1.017; // mass of thigh link
//     param_model->m_shank = 0.143; // mass of shank link
//     param_model->m_leg = param_model->m_thigh + param_model->m_shank;
//     param_model->m_total = param_model->m_trunk + 4 * param_model->m_leg;

//     param_model->Izz_thigh = 0.0057;     // MoI of thigh w.r.t. CoM
//     param_model->Izz_shank = 8.0318e-04; // MoI of shank w.r.t. CoM

//     param_model->Jzz_thigh = // 평행축 정리 적용
//         param_model->Izz_thigh + param_model->m_thigh * pow(param_model->d_thigh, 2); // MoI of thigh w.r.t. HFE
//     param_model->Jzz_shank = // 평행축 정리 적용
//         param_model->Izz_shank + param_model->m_shank * pow(param_model->d_shank, 2); // MoI of thigh w.r.t. KFE

//     double M1 = param_model->Jzz_thigh + param_model->m_shank * pow(param_model->L, 2); // A1 
//     double M2 = param_model->m_shank * param_model->d_shank * param_model->L * cos(state_model->q[1]); // A12 
//     double M12 = param_model->Jzz_shank; // A2

//     param_model->MatInertia_bi[0] = M1;
//     param_model->MatInertia_bi[1] = M12;
//     param_model->MatInertia_bi[2] = M12;
//     param_model->MatInertia_bi[3] = M2;

//     param_model->JzzR_thigh = param_model->Jzz_thigh + param_model->m_shank * pow(param_model->L, 2) +
//         param_model->Jzz_shank -
//         2 * param_model->m_shank * param_model->d_shank * param_model->L * cos(state_model->q[1]);
//     param_model->JzzR_couple =
//         param_model->Jzz_thigh + param_model->m_shank * pow(param_model->L, 2) - param_model->Jzz_shank;
//     param_model->JzzR_shank = param_model->Jzz_thigh + param_model->m_shank * pow(param_model->L, 2) +
//         param_model->Jzz_shank +
//         2 * param_model->m_shank * param_model->d_shank * param_model->L * cos(state_model->q[1]);

//     param_model->MatInertia_RW[0] =
//         param_model->JzzR_thigh / (4 * pow(param_model->L, 2) * pow(sin(state_model->q[1] / 2), 2));
//     param_model->MatInertia_RW[1] = param_model->JzzR_couple / (2 * pow(param_model->L, 2) * sin(state_model->q[1]));
//     param_model->MatInertia_RW[2] = param_model->JzzR_couple / (2 * pow(param_model->L, 2) * sin(state_model->q[1]));
//     param_model->MatInertia_RW[3] =
//         param_model->JzzR_shank / (4 * pow(param_model->L, 2) * pow(cos(state_model->q[1] / 2), 2));
// } // param_model parameter



// void model_param_cal(const mjModel* m, mjData* d, ParamModel_* param_model, StateModel_* state_model)
// {
//     /* Trunk Parameters */
//     param_model->m_hip = 2.5;
//     param_model->m_trunk_front = 10.;
//     param_model->m_trunk_rear = 18.;
//     param_model->m_trunk = 4 * param_model->m_hip + param_model->m_trunk_front + param_model->m_trunk_rear;

//     /* Leg Parameters */
//     param_model -> L = 0.25;
//     param_model -> d_thigh = 0.11017; // local position of CoM of thigh
//     param_model -> d_shank = 0.12997; // local position of CoM of shank

//     param_model->m_thigh = 1.017; // mass of thigh link
//     param_model->m_shank = 0.143; // mass of shank link
//     param_model->m_leg = param_model->m_thigh + param_model->m_shank;
//     param_model->m_total = param_model->m_trunk + 4 * param_model->m_leg;

//     param_model->Izz_thigh = 0.0057;     // MoI of thigh w.r.t. CoM
//     param_model->Izz_shank = 8.0318e-04; // MoI of shank w.r.t. CoM

    
    
// } // param_model parameter