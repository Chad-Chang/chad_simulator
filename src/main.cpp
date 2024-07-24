#include "funcs.h" // 사용자 정의 헤더파일 <>하면 안됨.
// namspace std
#include "actuator.h"
#include "rw_controller.h"
#include "kinematics.h"
#include "j_controller.h"
#include "trajectory.h"
// #include ""

RW_Controller leg_controller[4];
Kinematics leg_kinematics[4]; // 0: FL, 1: FR, 2: RL, 3: RR
Actuator ACT_RLHAA(5, 0);Actuator ACT_RLHIP(4, 0.546812);Actuator ACT_RLKNEE(3, 2.59478);
Actuator ACT_RRHAA(0, 0);Actuator ACT_RRHIP(1, 0.546812);Actuator ACT_RRKNEE(2, 2.59478);
Actuator ACT_FRHAA(11, 0);Actuator ACT_FRHIP(10, 0.546812);Actuator ACT_FRKNEE(9, 2.59478);
Actuator ACT_FLHAA(6, 0);Actuator ACT_FLHIP(7, 0.546812);Actuator ACT_FLKNEE(8, 2.59478);
trajectory leg_trajectory[4];

// Jacobian
Matrix2d J_FL;  Matrix2d J_FR;  Matrix2d J_RL;  Matrix2d J_RR;
Matrix2d J_T_FL;  Matrix2d J_T_FR;  Matrix2d J_T_RL;  Matrix2d J_T_RR;
Matrix2d J_T_inv_FL;  Matrix2d J_T_inv_FR;  Matrix2d J_T_inv_RL;  Matrix2d J_T_inv_RR;

// Controller output : RW PID INPUT//
MatrixXd leg_RWpid_output(2,4);

//trajectory
Matrix2d leg_RW_des_FL; // col 0 : position r, theta, col 1 : vel r, theta
Matrix2d leg_RW_des_FR; // col 0 : position r, theta, col 1 : vel r, theta
Matrix2d leg_RW_des_RL; // col 0 : position r, theta, col 1 : vel r, theta
Matrix2d leg_RW_des_RR; // col 0 : position r, theta, col 1 : vel r, theta
Vector2d DOB_data;


double r_des;


// posRW
MatrixXd leg_RWpos(2,4); // 0: FL, 1: FR, 2: RL, 3: RR
MatrixXd leg_RWpos_err(2,4); // 0: FL, 1: FR, 2: RL, 3: RR
MatrixXd leg_RWpos_err_old(2,4); // 0: FL, 1: FR, 2: RL, 3: RR

// velRW
MatrixXd leg_RWvel(2,4);
MatrixXd leg_RWvel_err(2,4);
MatrixXd leg_RWvel_err_old(2,4);


// Vector2d velRW_err;

Vector4d r_Pgain = {4000,40000,40000,40000};
Vector4d r_Igain = {0,0,0,0};
Vector4d r_Dgain = {200,0,0,0};
Vector4d th_Pgain = {4000,40000,40000,40000};
Vector4d th_Igain = {0,0,0,0};
Vector4d th_Dgain = {200,0,0,0};

// motor control input
MatrixXd leg_ctrl_input(2,4);

// lhs_DOB 
MatrixXd DOB_taubi_lhs(2,4); // col : 0~4 = FL FR RL RR
// DOB result
MatrixXd disturbance_hat(2,4);
// MatrixXd leg_ctrl_input_DOB(2,4);

Matrix2d inertia_jBi2BiTq_FL;  Matrix2d inertia_jBi2BiTq_FR; 
Matrix2d inertia_jBi2BiTq_RL;  Matrix2d inertia_jBi2BiTq_RR;

Matrix2d jnt2bi; Matrix2d bi2jnt;  Matrix2d tqbi2jtq;  Matrix2d jtq2tqbi;

Matrix2d DOB_Lambda_FL; Matrix2d DOB_Lambda_FR;
Matrix2d DOB_Lambda_RL; Matrix2d DOB_Lambda_RR;

Matrix2d FOB_Lambda_FL; Matrix2d FOB_Lambda_FR; 
Matrix2d FOB_Lambda_RL; Matrix2d FOB_Lambda_RR;

MatrixXd leg_motor_acc(2,4);
double motor_pos[3];

Vector2d disturbance;
double deri_cut = 200; double cut_off = 100;
double HAA_ctrl_input[4];
double gear_ratio = 1;
int t = 0;
int traj_t = 0;

void mycontroller(const mjModel* m, mjData* d)  // 제어주기 0.000025임
{   
    // leg_RW_des << 1,2,3,4,5,6,7,8,11,22,33,44,55,66,77,88;
    if(loop_index % 4 ==0) // sampling time 0.0001
    {   
        traj_t ++;
        t++;
        disturbance << sin(0.001* t), 0;
        // disturbance<< 1,0; // 바이아티큘러
    
        // joint PID gain setting  => j_set_gain(Pgain, Igain, Dgain) 
        for(int i = 0 ; i<4;i++)
        {   
            leg_controller[i].j_set_gain(100,0,1);
            leg_controller[i].rw_set_gain(r_Pgain,r_Igain, r_Dgain, th_Pgain, th_Igain, th_Dgain);
            leg_controller[i].j_setDelayData();
            leg_controller[i].rw_setDelayData();
            // leg_controller[i].FOBRW(); // 인스턴스 내부에서 force값 저장

            leg_kinematics[i].set_DelayDATA();
        }

         /****************** joint angle read + joint delay + calculate acc******************/ 
        
//         /****************** Trajectory ******************/
        // col0 : pos, col1 : vel
        // leg_RW_des_FL = leg_trajectory[0].SLIP_traj();
        // leg_RW_des_FR = leg_trajectory[0].SLIP_traj();
        // leg_RW_des_RL = leg_trajectory[0].SLIP_traj();
        // leg_RW_des_RR = leg_trajectory[0].SLIP_traj();

        r_des = leg_kinematics[0].pos_trajectory(traj_t, 0); // temporal trajectory
        leg_RWpos_err.col(0) = leg_kinematics[0].get_posRW_error(0);
        leg_RWpos_err_old.col(0) = leg_kinematics[0].get_posRW_error(1);
        leg_RWpid_output.col(0)(0) = leg_controller[0].rw_posPID(leg_RWpos_err.col(0), leg_RWpos_err_old.col(0), 0, 0); // rw force
            leg_RWpid_output.col(0)(1) = leg_controller[0].rw_posPID(leg_RWpos_err.col(0), leg_RWpos_err_old.col(0), 1, 0); // PID 출력
        leg_ctrl_input.col(0) = gear_ratio * J_T_FL * leg_RWpid_output.col(0);//+ disturbance_hat.col(0);  // biarticular torque
        
        // // joint space acceleration 계산
        HAA_ctrl_input[0] = leg_controller[0].j_posPID(0,ACT_FLHAA.getMotor_pos(),T,cutoff);
        HAA_ctrl_input[1] = leg_controller[1].j_posPID(0,ACT_FRHAA.getMotor_pos(),T,cutoff);
        HAA_ctrl_input[2] = leg_controller[2].j_posPID(0,ACT_RLHAA.getMotor_pos(),T,cutoff);
        HAA_ctrl_input[3] = leg_controller[3].j_posPID(0,ACT_RRHAA.getMotor_pos(),T,cutoff);


        
        // disturbance = tqbi2jtq*disturbance;
        // cout << disturbance<<endl;
        ACT_FLHAA.DATA_Send(d,HAA_ctrl_input[0]);
        ACT_FLHIP.DATA_Send(d,leg_ctrl_input.col(0)(0)+ leg_ctrl_input.col(0)(1) + disturbance[0]);
        ACT_FLKNEE.DATA_Send(d,leg_ctrl_input.col(0)(1)) ;

        ACT_FRHAA.DATA_Send(d,HAA_ctrl_input[1]);
        ACT_FRHIP.DATA_Send(d,leg_ctrl_input.col(1)(0)+ leg_ctrl_input.col(1)(1));
        ACT_FRKNEE.DATA_Send(d,leg_ctrl_input.col(1)(1));

        ACT_RLHAA.DATA_Send(d,HAA_ctrl_input[2]);
        ACT_RLHIP.DATA_Send(d,leg_ctrl_input.col(2)(0)+ leg_ctrl_input.col(2)(1));
        ACT_RLKNEE.DATA_Send(d,leg_ctrl_input.col(2)(1));

        ACT_RRHAA.DATA_Send(d,HAA_ctrl_input[3]);
        ACT_RRHIP.DATA_Send(d,leg_ctrl_input.col(3)(0)+ leg_ctrl_input.col(3)(1));
        ACT_RRKNEE.DATA_Send(d,leg_ctrl_input.col(3)(1));

        ACT_FLHAA.DATA_Receive(d);ACT_FLHIP.DATA_Receive(d);ACT_FLKNEE.DATA_Receive(d);
        ACT_FRHAA.DATA_Receive(d);ACT_FRHIP.DATA_Receive(d);ACT_FRKNEE.DATA_Receive(d);
        ACT_RLHAA.DATA_Receive(d);ACT_RLHIP.DATA_Receive(d);ACT_RLKNEE.DATA_Receive(d);
        ACT_RRHAA.DATA_Receive(d);ACT_RRHIP.DATA_Receive(d);ACT_RRKNEE.DATA_Receive(d);
        motor_pos[0] = ACT_FLHIP.getMotor_pos();
        motor_pos[1] = ACT_FLHIP.getMotor_vel();
        motor_pos[2] = ACT_FLHIP.getMotor_acc();

        // leg motor acc
        leg_motor_acc.col(0) << ACT_FLHIP.getMotor_acc(),ACT_FLKNEE.getMotor_acc();
        leg_motor_acc.col(1) << ACT_FLHIP.getMotor_acc(),ACT_FLKNEE.getMotor_acc();
        leg_motor_acc.col(2) << ACT_FLHIP.getMotor_acc(),ACT_FLKNEE.getMotor_acc();
        leg_motor_acc.col(3) << ACT_FLHIP.getMotor_acc(),ACT_FLKNEE.getMotor_acc();
        // cout << leg_motor_acc.col(0)<< endl;
        for(int i =0 ; i <4; i++)
            leg_motor_acc.col(i) = jnt2bi*leg_motor_acc.col(i);
        
         ////////////////////// 시리얼 구조일때  - mujoco ////////////////////////

        leg_kinematics[0].Cal_RW(ACT_FLHIP.getMotor_pos(), ACT_FLKNEE.getMotor_pos()+ACT_FLHIP.getMotor_pos(),ACT_FLHIP.getMotor_vel(), ACT_FLKNEE.getMotor_vel()+ACT_FLHIP.getMotor_vel(),0); 
        leg_kinematics[1].Cal_RW(ACT_FRHIP.getMotor_pos(), ACT_FRKNEE.getMotor_pos()+ACT_FRHIP.getMotor_pos(),ACT_FRHIP.getMotor_vel(), ACT_FRKNEE.getMotor_vel()+ACT_FRHIP.getMotor_vel(),1);
        leg_kinematics[2].Cal_RW(ACT_RLHIP.getMotor_pos(), ACT_RLKNEE.getMotor_pos()+ACT_RLHIP.getMotor_pos(),ACT_RLHIP.getMotor_vel(), ACT_RLKNEE.getMotor_vel()+ACT_RLHIP.getMotor_vel(),2);
        leg_kinematics[3].Cal_RW(ACT_RRHIP.getMotor_pos(), ACT_RRKNEE.getMotor_pos()+ACT_RRHIP.getMotor_pos(),ACT_RRHIP.getMotor_vel(), ACT_RRKNEE.getMotor_vel()+ACT_RRHIP.getMotor_vel(),3);
        

        // DOB lambda : biarticular inertia
        DOB_Lambda_FL =  leg_kinematics[0].get_Lamda_nominal_DOB(); DOB_Lambda_FR =  leg_kinematics[1].get_Lamda_nominal_DOB(); // 
        DOB_Lambda_RL =  leg_kinematics[2].get_Lamda_nominal_DOB(); DOB_Lambda_RR =  leg_kinematics[3].get_Lamda_nominal_DOB();

        // FOB lambda : RW
        FOB_Lambda_FL =  leg_kinematics[0].get_Lamda_nominal_FOB(); FOB_Lambda_FR =  leg_kinematics[1].get_Lamda_nominal_FOB();
        FOB_Lambda_RL =  leg_kinematics[2].get_Lamda_nominal_FOB(); FOB_Lambda_RR =  leg_kinematics[3].get_Lamda_nominal_FOB();
          
          //jacobian
        J_FL = leg_kinematics[0].get_RW_Jacobian(); J_FR = leg_kinematics[1].get_RW_Jacobian();
        J_RL = leg_kinematics[2].get_RW_Jacobian(); J_RR = leg_kinematics[3].get_RW_Jacobian();
        J_T_FL = leg_kinematics[0].get_RW_Jacobian_Trans();  J_T_FR = leg_kinematics[1].get_RW_Jacobian_Trans();
        J_T_RL = leg_kinematics[2].get_RW_Jacobian_Trans();  J_T_RR = leg_kinematics[3].get_RW_Jacobian_Trans();
        J_T_inv_FL = leg_kinematics[0].get_RW_Jacobian_Trans_inv();  J_T_FR = leg_kinematics[1].get_RW_Jacobian_Trans_inv();
        J_T_inv_RL = leg_kinematics[2].get_RW_Jacobian_Trans_inv();  J_T_RR = leg_kinematics[3].get_RW_Jacobian_Trans_inv();

        leg_kinematics[0].model_param_cal(ACT_FLHIP.getMotor_pos(), ACT_FLKNEE.getMotor_pos()+ACT_FLHIP.getMotor_pos()); // 현재 포즈에 맞는 inertia 계산
        leg_kinematics[1].model_param_cal(ACT_FRHIP.getMotor_pos(), ACT_FRKNEE.getMotor_pos()+ACT_FRHIP.getMotor_pos());
        leg_kinematics[2].model_param_cal(ACT_RLHIP.getMotor_pos(), ACT_RLKNEE.getMotor_pos()+ACT_RLHIP.getMotor_pos());
        leg_kinematics[3].model_param_cal(ACT_RRHIP.getMotor_pos(), ACT_RRKNEE.getMotor_pos()+ACT_RRHIP.getMotor_pos());

        // if(2 <d-> time && d-> time < 2.001) {
            disturbance_hat.col(0) = leg_controller[0].DOBRW(leg_ctrl_input.col(0),DOB_Lambda_FL, leg_motor_acc.col(0),300,1);
            DOB_data = disturbance_hat.col(0);
            // }
        


}
    if (loop_index % data_frequency == 0) {     // loop_index를 data_frequency로 나눈 나머지가 0이면 데이터를 저장.
        save_data(m, d);
        
    }
    loop_index = loop_index + 1;
}

// main function
int main(int argc, const char** argv)
{   
    for(int i = 0 ; i<4 ; i++)
        leg_controller[i].init();   
    tqbi2jtq << 1, 1, 0, 1; jtq2tqbi << 1,-1,0,1;
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
    
 
    d->qpos[7] = 0; //FLHAA         //d->ctrl[0] FLHAA
    d->qpos[8] = pi/4; //FLHIP       //d->ctrl[1] FLHIP
    d->qpos[9] = pi/2; //FLKNEE        //d->ctrl[2] FLKNEE
    d->qpos[10] = 0; //FRHAA        //d->ctrl[3] FRHAA
    d->qpos[11] = pi/4; //FRHIP        //d->ctrl[4] FRHIP
    d->qpos[12] = pi/2; //FRKNEE       //d->ctrl[5] FRKNEE
    d->qpos[13] = 0; //RLHAA        //d->ctrl[6] RLHAA
    d->qpos[14] = pi/4; //RLHIP        //d->ctrl[7] RLHIP
    d->qpos[15] = pi/2; //RLKNEE       //d->ctrl[8] RLKNEE
    d->qpos[16] = 0; //RRHAA        //d->ctrl[9] RRHAA
    d->qpos[17] = pi/4; //RRHIP        //d->ctrl[10] RRHIP
    d->qpos[18] = pi/2; //RRKNEE       //d->ctrl[11] RRKNEE
    

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
