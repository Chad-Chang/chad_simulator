#include "funcs.h" // 사용자 정의 헤더파일 <>하면 안됨.
// namspace std
#include "actuator.h"
#include "rw_controller.h"
#include "kinematics.h"
#include "j_controller.h"


RW_Controller C_FL;RW_Controller C_FR;RW_Controller C_RL;RW_Controller C_RR;
// Jacobian
Matrix2d J_FL;Matrix2d J_FR;Matrix2d J_RL;Matrix2d J_RR;
Matrix2d JTrans_FL;Matrix2d JTrans_FR;Matrix2d JTrans_RL;Matrix2d JTrans_RR;
// Controller output : RW PID INPUT//
Vector2d FL_output;Vector2d FR_output;Vector2d RL_output;Vector2d RR_output;
Actuator ACT_RLHAA(5, 0);Actuator ACT_RLHIP(4, 0.546812);Actuator ACT_RLKNEE(3, 2.59478);
Actuator ACT_RRHAA(0, 0);Actuator ACT_RRHIP(1, 0.546812);Actuator ACT_RRKNEE(2, 2.59478);
Actuator ACT_FRHAA(11, 0);Actuator ACT_FRHIP(10, 0.546812);Actuator ACT_FRKNEE(9, 2.59478);
Actuator ACT_FLHAA(6, 0);Actuator ACT_FLHIP(7, 0.546812);Actuator ACT_FLKNEE(8, 2.59478);

Kinematics K_FL;Kinematics K_FR;Kinematics K_RL;Kinematics K_RR;
// posRW
Vector2d posRW_FL;Vector2d posRW_FR;Vector2d posRW_RL;Vector2d posRW_RR;
//Vector2d velRW;
Vector2d posRW_err_FL;Vector2d posRW_err_FR;Vector2d posRW_err_RL;Vector2d posRW_err_RR;
Vector2d posRW_err_old_FL;Vector2d posRW_err_old_FR;Vector2d posRW_err_old_RL;Vector2d posRW_err_old_RR;
Vector2d velRW_err;

Vector4d r_Pgain = {40000,40000,40000,40000};
Vector4d r_Igain = {0,0,0,0};
Vector4d r_Dgain = {0,0,0,0};
Vector4d th_Pgain = {40000,40000,40000,40000};
Vector4d th_Igain = {0,0,0,0};
Vector4d th_Dgain = {0,0,0,0};

Vector2d th_acc_FL; Vector2d th_acc_FR; Vector2d th_acc_RL; Vector2d th_acc_RR;
Vector2d th_acc_FL_old; Vector2d th_acc_FR_old; Vector2d th_acc_RL_old; Vector2d th_acc_RR_old;
Vector2d th_vel_FL; Vector2d th_vel_FR; Vector2d th_vel_RL; Vector2d th_vel_RR;
Vector2d th_vel_FL_old; Vector2d th_vel_FR_old; Vector2d th_vel_RL_old; Vector2d th_vel_RR_old;

// motor control input
Vector2d FL_ctrl_input;Vector2d FR_ctrl_input;Vector2d RL_ctrl_input;Vector2d RR_ctrl_input; // PID

Vector2d FL_ctrl_input_DOB;Vector2d FR_ctrl_input_DOB;Vector2d RL_ctrl_input_DOB;Vector2d RR_ctrl_input_DOB; // dob보상까지 

Matrix2d inertia_jBi2BiTq_FL; Matrix2d inertia_jBi2BiTq_FR; Matrix2d inertia_jBi2BiTq_RL; Matrix2d inertia_jBi2BiTq_RR;

Matrix2d jnt2bi; Matrix2d bi2jnt;

Vector2d FL_torq_hat; Vector2d FR_torq_hat; Vector2d RL_torq_hat; Vector2d RR_torq_hat;

Vector2d FL_d_hat; Vector2d FR_d_hat; Vector2d RL_d_hat;  Vector2d RR_d_hat;
Vector2d FL_d_hat_old; Vector2d FR_d_hat_old; Vector2d RL_d_hat_old; Vector2d RR_d_hat_old;
extern Vector2d QFL_d_hat; extern Vector2d QFR_d_hat; extern Vector2d QRL_d_hat;  extern Vector2d QRR_d_hat;
Vector2d QFL_d_hat_old; Vector2d QFR_d_hat_old; Vector2d QRL_d_hat_old; Vector2d QRR_d_hat_old;

double deri_cut = 200; double cut_off = 100;
double HAA_control_input[4];
double gear_ratio = 1;
int t = 0;
int traj_t = 0;

void mycontroller(const mjModel* m, mjData* d)  // 제어주기 0.000025임
{   
    
    if(loop_index % 4 ==0) // sampling time 0.0001
    {   
        th_vel_FL_old(0) = th_vel_FL(0); th_vel_FL_old(1) = th_vel_FL(1);
        th_vel_FR_old(0) = th_vel_FR(0); th_vel_FR_old(1) = th_vel_FR(1);
        th_vel_RL_old(0) = th_vel_RL(0); th_vel_RL_old(1) = th_vel_RL(1);
        th_vel_RR_old(0) = th_vel_RR(0); th_vel_RR_old(1) = th_vel_RR(1);

        th_acc_FL_old(0) = th_acc_FL(0); th_acc_FL_old(1) = th_acc_FL(1);
        th_acc_FR_old(0) = th_acc_FR(0); th_acc_FR_old(1) = th_acc_FR(1);
        th_acc_RL_old(0) = th_acc_RL(0); th_acc_RL_old(1) = th_acc_RL(1);
        th_acc_RR_old(0) = th_acc_RR(0); th_acc_RR_old(1) = th_acc_RR(1);

        FL_d_hat_old = FL_d_hat; FR_d_hat_old = FR_d_hat; 
        RL_d_hat_old = RL_d_hat; RR_d_hat_old = RR_d_hat; 


        // joint PID gain setting  => j_set_gain(Pgain, Igain, Dgain) 
        C_FL.j_set_gain(100,0,1);C_FR.j_set_gain(100,0,1);C_RL.j_set_gain(100,0,1);C_RR.j_set_gain(100,0,1); // joint controller
        C_FL.rw_set_gain(r_Pgain,r_Igain, r_Dgain, th_Pgain, th_Igain, th_Dgain);
        C_FR.rw_set_gain(r_Pgain,r_Igain, r_Dgain, th_Pgain, th_Igain, th_Dgain);
        C_RL.rw_set_gain(r_Pgain,r_Igain, r_Dgain, th_Pgain, th_Igain, th_Dgain);
        C_RR.rw_set_gain(r_Pgain,r_Igain, r_Dgain, th_Pgain, th_Igain, th_Dgain);

        traj_t ++;
        t++;
        C_FL.j_setDelayData();C_FR.j_setDelayData();C_RL.j_setDelayData();C_RR.j_setDelayData();
        C_FL.rw_setDelayData(); C_FR.rw_setDelayData(); C_RL.rw_setDelayData(); C_RR.rw_setDelayData();
        K_FL.set_DelayDATA();K_FR.set_DelayDATA();K_RL.set_DelayDATA();K_RR.set_DelayDATA();
        
        /****************** joint angle read ******************/
        ACT_FLHAA.DATA_Receive(d);ACT_FLHIP.DATA_Receive(d);ACT_FLKNEE.DATA_Receive(d);
        ACT_FRHAA.DATA_Receive(d);ACT_FRHIP.DATA_Receive(d);ACT_FRKNEE.DATA_Receive(d);
        ACT_RLHAA.DATA_Receive(d);ACT_RLHIP.DATA_Receive(d);ACT_RLKNEE.DATA_Receive(d);
        ACT_RRHAA.DATA_Receive(d);ACT_RRHIP.DATA_Receive(d);ACT_RRKNEE.DATA_Receive(d);

        th_vel_FL(0) = ACT_FLHIP.getMotor_vel(); th_vel_FL(1) = ACT_FLKNEE.getMotor_vel();
        th_vel_FR(0) = ACT_FRHIP.getMotor_vel(); th_vel_FR(1) = ACT_FRKNEE.getMotor_vel();
        th_vel_RL(0) = ACT_RLHIP.getMotor_vel(); th_vel_RL(1) = ACT_RLKNEE.getMotor_vel();
        th_vel_RR(0) = ACT_RRHIP.getMotor_vel(); th_vel_RR(1) = ACT_RRKNEE.getMotor_vel();

        // 가속도 구하기 
        th_acc_FL(0) = tustin_derivative(th_vel_FL(0),th_vel_FL_old(0),th_acc_FL_old(0),deri_cut);
        th_acc_FL(1) = tustin_derivative(th_vel_FL(1),th_vel_FL_old(1),th_acc_FL_old(1),deri_cut);

        th_acc_FR(0) = tustin_derivative(th_vel_FR(0),th_vel_FR_old(0),th_acc_FR_old(0),deri_cut);
        th_acc_FR(1) = tustin_derivative(th_vel_FR(1),th_vel_FR_old(1),th_acc_FR_old(1),deri_cut);

        th_acc_RL(0) = tustin_derivative(th_vel_RL(0),th_vel_RL_old(0),th_acc_RL_old(0),deri_cut);
        th_acc_RL(1) = tustin_derivative(th_vel_RL(1),th_vel_RL_old(1),th_acc_RL_old(1),deri_cut);

        th_acc_RR(0) = tustin_derivative(th_vel_RR(0),th_vel_RR_old(0),th_acc_RR_old(0),deri_cut);
        th_acc_RR(1) = tustin_derivative(th_vel_RR(1),th_vel_RR_old(1),th_acc_RR_old(1),deri_cut);

        /****************** Kinematics ******************/
        ////////////////////// 바이아티큘러 구조일때 - real robot////////////////////////
        // K_FL.Cal_RW(ACT_FLHIP.getMotor_pos(), ACT_FLKNEE.getMotor_pos(),0); 
        // K_FR.Cal_RW(ACT_FRHIP.getMotor_pos(), ACT_FRKNEE.getMotor_pos(),1);
        // K_RL.Cal_RW(ACT_RLHIP.getMotor_pos(), ACT_RLKNEE.getMotor_pos(),2);
        // K_RR.Cal_RW(ACT_RRHIP.getMotor_pos(), ACT_RRKNEE.getMotor_pos(),3);
        
        

        ////////////////////// 시리얼 구조일때  - mujoco ////////////////////////
        K_FL.Cal_RW(ACT_FLHIP.getMotor_pos(), ACT_FLKNEE.getMotor_pos()+ACT_FLHIP.getMotor_pos(),ACT_FLHIP.getMotor_vel(), ACT_FLKNEE.getMotor_vel()+ACT_FLHIP.getMotor_vel(),0); 
        K_FR.Cal_RW(ACT_FRHIP.getMotor_pos(), ACT_FRKNEE.getMotor_pos()+ACT_FRHIP.getMotor_pos(),ACT_FRHIP.getMotor_vel(), ACT_FRKNEE.getMotor_vel()+ACT_FRHIP.getMotor_vel(),1);
        K_RL.Cal_RW(ACT_RLHIP.getMotor_pos(), ACT_RLKNEE.getMotor_pos()+ACT_RLHIP.getMotor_pos(),ACT_RLHIP.getMotor_vel(), ACT_RLKNEE.getMotor_vel()+ACT_RLHIP.getMotor_vel(),2);
        K_RR.Cal_RW(ACT_RRHIP.getMotor_pos(), ACT_RRKNEE.getMotor_pos()+ACT_RRHIP.getMotor_pos(),ACT_RRHIP.getMotor_vel(), ACT_RRKNEE.getMotor_vel()+ACT_RRHIP.getMotor_vel(),3);
        
          
        J_FL = K_FL.get_RW_Jacobian();J_FR = K_FR.get_RW_Jacobian();J_RL = K_RL.get_RW_Jacobian();J_RR= K_RR.get_RW_Jacobian();
          
        JTrans_FL = K_FL.get_RW_Jacobian_Trans();JTrans_FR = K_FR.get_RW_Jacobian_Trans();JTrans_RL = K_RL.get_RW_Jacobian_Trans();JTrans_RR = K_RR.get_RW_Jacobian_Trans();

        /****************** Trajectory ******************/
        
        K_FL.pos_trajectory(traj_t, 0); K_FR.pos_trajectory(traj_t, 1); K_RL.pos_trajectory(traj_t, 2); K_RR.pos_trajectory(traj_t, 3); 

        /****************** State ******************/ // pos RW
        
        posRW_FL = K_FL.get_posRW(); // function의 마지막 input이 0이면 현재 값, 1이면 이전 값
        posRW_FR = K_FR.get_posRW();
        posRW_RL = K_RL.get_posRW();
        posRW_RR = K_RR.get_posRW();
        
        /****************** State error ******************/ // index 0: curr error/ index 1 : old error
        posRW_err_FL = K_FL.get_posRW_error(0);
        posRW_err_FR = K_FR.get_posRW_error(0);
        posRW_err_RL = K_RL.get_posRW_error(0);
        posRW_err_RR = K_RR.get_posRW_error(0);

        /****************** State error old******************/ // index 0: curr error/ index 1 : old error
        posRW_err_old_FL = K_FL.get_posRW_error(1);
        posRW_err_old_FR = K_FR.get_posRW_error(1);
        posRW_err_old_RL = K_RL.get_posRW_error(1);
        posRW_err_old_RR = K_RR.get_posRW_error(1);
        

        /****************** Conrtoller ******************/ // index [0] : R direction output, index [1] : th direction output
        FL_output[0] = C_FL.rw_posPID(posRW_err_FL, posRW_err_old_FL, 0, 0);
        FL_output[1] = C_FL.rw_posPID(posRW_err_FL, posRW_err_old_FL, 1, 0);
        
        FR_output[0] = C_FR.rw_posPID(posRW_err_FR, posRW_err_old_FR, 0, 1);
        FR_output[1] = C_FR.rw_posPID(posRW_err_FR, posRW_err_old_FR, 1, 1);

        RL_output[0] = C_RL.rw_posPID(posRW_err_RL, posRW_err_old_RL, 0, 2); // R direction output
        RL_output[1] = C_RL.rw_posPID(posRW_err_RL, posRW_err_old_RL, 1, 2); // th direction output
                
        RR_output[0] = C_RR.rw_posPID(posRW_err_RR, posRW_err_old_RR, 0, 3);
        RR_output[1] = C_RR.rw_posPID(posRW_err_RR, posRW_err_old_RR, 1, 3);
        
        
        /****************** Put the torque in Motor ******************/ // 믈리량 tm, tb
        FL_ctrl_input =  gear_ratio * JTrans_FL * FL_output; FR_ctrl_input = gear_ratio * JTrans_FR * FR_output;
        RL_ctrl_input =  gear_ratio * JTrans_RL * RL_output; RR_ctrl_input = gear_ratio * JTrans_RR * RR_output;
        

        disturbance = 1*sin(0.001*t);

        ACT_FLHAA.DATA_Send(d,HAA_control_input[0]);
        ACT_FLHIP.DATA_Send(d,FL_ctrl_input[0]+ FL_ctrl_input[1]+disturbance);
        ACT_FLKNEE.DATA_Send(d,FL_ctrl_input[1]);
        ACT_FRHAA.DATA_Send(d,HAA_control_input[1]);
        ACT_FRHIP.DATA_Send(d,FR_ctrl_input[0]+FR_ctrl_input[1]+disturbance);
        ACT_FRKNEE.DATA_Send(d,FR_ctrl_input[1]);
        ACT_RLHAA.DATA_Send(d,HAA_control_input[2]);
        ACT_RLHIP.DATA_Send(d,RL_ctrl_input[0]+RL_ctrl_input[1]+disturbance);
        ACT_RLKNEE.DATA_Send(d,RL_ctrl_input[1]);
        ACT_RRHAA.DATA_Send(d,HAA_control_input[3]);
        ACT_RRHIP.DATA_Send(d,RR_ctrl_input[0]+RR_ctrl_input[1]+disturbance);
        ACT_RRKNEE.DATA_Send(d,RR_ctrl_input[1]);
        // joint space acceleration 계산


        /*******************HAA position control input*******************/

        // HAA_control_input[0] = C_FL.j_posPID(PI/6,ACT_FLHAA.getMotor_pos(),T,cutoff);
        // HAA_control_input[1] = C_FR.j_posPID(PI/2,ACT_FRHAA.getMotor_pos(),T,cutoff);
        // HAA_control_input[2] = C_RL.j_posPID(PI/3,ACT_RLHAA.getMotor_pos(),T,cutoff);
        // HAA_control_input[3] = C_RR.j_posPID(PI/10,ACT_RRHAA.getMotor_pos(),T,cutoff);
        HAA_control_input[0] = C_FL.j_posPID(0,ACT_FLHAA.getMotor_pos(),T,cutoff);
        HAA_control_input[1] = C_FR.j_posPID(0,ACT_FRHAA.getMotor_pos(),T,cutoff);
        HAA_control_input[2] = C_RL.j_posPID(0,ACT_RLHAA.getMotor_pos(),T,cutoff);
        HAA_control_input[3] = C_RR.j_posPID(0,ACT_RRHAA.getMotor_pos(),T,cutoff);
        
        // d-> qpos[2] = 0.5;
        // 입력 토크
    // 
        // cout << "FR "<< disturbance<< " " << QFR_d_hat(0)<< " "<<QFR_d_hat(1) << endl;
        // cout << "FL "<<disturbance<< " " << QFL_d_hat(0)<< " "<<QFL_d_hat(1) << endl;
        // cout << "RR "<< disturbance<< " " << QRR_d_hat(0)<< " "<<QRR_d_hat(1) << endl;

    }
        
        

    if (loop_index % data_frequency == 0) {     // loop_index를 data_frequency로 나눈 나머지가 0이면 데이터를 저장.
        save_data(m, d);
        
    }
    //loop_index += 1;
    loop_index = loop_index + 1;
}

// main function
int main(int argc, const char** argv)
{
    // 필요한 행렬
    jnt2bi << 1,0,
        1,1;
    bi2jnt << 1,0,
        -1,1;

    // activate software
    mj_activate("mjkey.txt");

    
    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line argumentsgear_ratio
    if (argc < 2)
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if (strlen(argv[1]) > 4 && !strcmp(argv[1] + strlen(argv[1]) - 4, ".mjb"))
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if (!m)
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    //double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 0.000000};
    double arr_view[] = { 90, -5, 5, 0.012768, -0.000000, 1.254336 };
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];
    
    // qpos is dim xq x 1 = 7 x 1; 3 translations + 4 quaternions
    
    // custom controller
    mjcb_control = mycontroller; // 무한 반복되는 함수

    fid = fopen(datafile, "w");
    init_save_data();

// 초기 각도 입력    0.546812);Actuator ACT_RLKNEE(3, 2.59478)
    
   d->qpos[8] = 0.546812;
   d->qpos[9] = 2.59478;
   d->qpos[11] = 0.546812;
   d->qpos[12] = 2.59478;
   d->qpos[14] = 0.546812;
   d->qpos[15] = 2.59478;
   d->qpos[17] = 0.546812;
   d->qpos[18] = 2.59478;


    // use the first while condition if you want to simulate for a period.
    while (!glfwWindowShouldClose(window)) // 주기 : 0.0167
    {
    

        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        
        mjtNum simstart = d->time;
        

        // 여기가 loop time 0.0001인 부분 -> update하는 부분을 넣으면 되는 부분임
        while (d->time - simstart < 1.0 / 60.0) 
        {
            
            mj_step(m, d);
            // printf("err : %f, err_old : %f, err-err_old : %f\n",err,err_old, err-err_old);        
        }
        // printf("%f \n ", d -> time - simstart);
        // if (d->time >= simend) {
        //     fclose(fid);
        //     break;
        // }

        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        //opt.frame = mjFRAME_WORLD;
        //cam.lookat[0] = d->qpos[0];
        //cam.lookat[1] = 0;
        //cam.lookat[2] = 0;
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}
