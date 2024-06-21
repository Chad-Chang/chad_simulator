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
char datafile[] = "data/ROBOT_DOB.csv";
double disturbance;

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
Vector2d FL_distub; Vector2d FR_distub; Vector2d RL_distub;  Vector2d RR_distub;
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
    //write name of the variable here (header)
    fprintf(fid, "t, ");
    fprintf(fid, "dist, FL_hat, FR_hat, RL_hat, RR_hat"); // disturbace error

    //Don't remove the newline
    fprintf(fid, "\n");
}

////***************************
////This function is called at a set frequency, put data here
void save_data(const mjModel* m, mjData* d) 
{   
    //data here should correspond to headers in init_save_data()
    //seperate data by a space %f followed by space
    fprintf(fid, "%f, ", d->time);
    fprintf(fid, "%f, %f, %f, %f, %f,", disturbance,FL_distub(0),FR_distub(0),RL_distub(0),RR_distub(0)); // hip에서만 측정
    
    //Don't remove the newline
    fprintf(fid, "\n");
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
