// #pragma once
#include "controller.h"
#include "actuator.h"
#include "kinematics.h"
#include "mj_preset.h" // 사용자 정의 헤더파일 <>하면 안됨




using namespace std;
const double ground_level = 0.0;

int loop_time_1ms = 4; // 0.0001ms초를 보장해주는 놈

Controller C_FL; Controller C_FR; Controller C_RL; Controller C_RR;

// //다리별 자코비안
Matrix2d J_FL;Matrix2d J_FR;Matrix2d J_RL;Matrix2d J_RR;

// //다리별 자코비안T
Matrix2d JTrans_FL;Matrix2d JTrans_FR;Matrix2d JTrans_RL;Matrix2d JTrans_RR;

//FL controller output : 2x1
Vector2d FL_output;Vector2d FR_output;Vector2d RL_output;Vector2d RR_output; 

// 모터 level control input : 2x1 
Vector2d FL_control_input; Vector2d FR_control_input; Vector2d RL_control_input; Vector2d RR_control_input;

//actuator 선언
Actuator ACT_RLHAA(5, 0); Actuator ACT_RLHIP(4, 0.546812); Actuator ACT_RLKNEE(3, 2.59478);
Actuator ACT_RRHAA(0, 0);Actuator ACT_RRHIP(1, 0.546812);Actuator ACT_RRKNEE(2, 2.59478);
Actuator ACT_FRHAA(9, 0);Actuator ACT_FRHIP(10, 0.546812);Actuator ACT_FRKNEE(11, 2.59478);
Actuator ACT_FLHAA(6, 0);Actuator ACT_FLHIP(7, 0.546812);Actuator ACT_FLKNEE(8, 2.59478);

// kinematics : 다리별
Kinematics K_FL;Kinematics K_FR;Kinematics K_RL;Kinematics K_RR;

int t = 0;
int traj_t = 0;


// posRW : 2x1
Vector2d posRW_FL;Vector2d posRW_FR;Vector2d posRW_RL;Vector2d posRW_RR;
Vector2d posRW_err_FL;Vector2d posRW_err_FR;Vector2d posRW_err_RL;Vector2d posRW_err_RR;
Vector2d posRW_err_old_FL;Vector2d posRW_err_old_FR;Vector2d posRW_err_old_RL;Vector2d posRW_err_old_RR;
int t_end = 0; 

void mycontroller(const mjModel* m, mjData* d)  // 제어주기 0.000025임
{      
    // if(d->sensordata[46]<ground_level) d->qpos[2]+=0.01;
    // if(d->sensordata[49]<ground_level) d->qpos[2]+=0.01;
    // if(d->sensordata[52]<ground_level) d->qpos[2]+=0.01;
    // if(d->sensordata[55]<ground_level) d->qpos[2]+=0.01;



    if(loop_index % loop_time_1ms == 0) // 1ms 맞추기
    {   
        t ++;
        printf("time _start= %d , time _end = %d \n",t, t_end);
        /**************** Controller DATA set길이 : update ***************/
        C_FL.setDelayData(); C_FR.setDelayData();C_RL.setDelayData();C_RR.setDelayData();
        /**************** Kinematic error update ***************/
        K_FL.set_DelayDATA();K_FR.set_DelayDATA();K_RL.set_DelayDATA();K_RR.set_DelayDATA();
        /**************** Data receive from Mjdata ***************/ // qpos읽음
        ACT_FLHAA.DATA_Receive(m,d);ACT_FLHIP.DATA_Receive(m,d);ACT_FLKNEE.DATA_Receive(m,d);
        ACT_FRHAA.DATA_Receive(m,d);ACT_FRHIP.DATA_Receive(m,d);ACT_FRKNEE.DATA_Receive(m,d);
        ACT_RLHAA.DATA_Receive(m,d);ACT_RLHIP.DATA_Receive(m,d);ACT_RLKNEE.DATA_Receive(m,d);
        ACT_RRHAA.DATA_Receive(m,d);ACT_RRHIP.DATA_Receive(m,d);ACT_RRKNEE.DATA_Receive(m,d);


        /****************** Kinematics ******************/
        
        K_FL.Cal_RW(ACT_FLHIP.getMotor_pos(), ACT_FLKNEE.getMotor_pos(),0); K_FR.Cal_RW(ACT_FRHIP.getMotor_pos(), ACT_FRKNEE.getMotor_pos(),1);
        K_RL.Cal_RW(ACT_RLHIP.getMotor_pos(), ACT_RLKNEE.getMotor_pos(),2); K_RR.Cal_RW(ACT_RRHIP.getMotor_pos(), ACT_RRKNEE.getMotor_pos(),3);
          
        J_FL = K_FL.get_RW_Jacobian(); J_FR = K_FR.get_RW_Jacobian(); J_RL = K_RL.get_RW_Jacobian();J_RR= K_RR.get_RW_Jacobian();
          
        JTrans_FL = K_FL.get_RW_Jacobian_Trans();JTrans_FR = K_FR.get_RW_Jacobian_Trans();JTrans_RL = K_RL.get_RW_Jacobian_Trans();JTrans_RR = K_RR.get_RW_Jacobian_Trans();


        /****************** Trajectory ******<< " "************/
        // FL, FR, RL, RR
        K_FL.pos_trajectory(traj_t, 0); K_FR.pos_trajectory(traj_t, 1);K_RL.pos_trajectory(traj_t, 2);K_RR.pos_trajectory(traj_t, 3);
        
        /****************** state update ******************/
        posRW_FL = K_FL.get_posRW(); posRW_FR = K_FR.get_posRW();posRW_RL = K_RL.get_posRW(); posRW_RR = K_RR.get_posRW();

        
        /****************** State error old******************/ // index 0: curr error/ index 1 : old error
        posRW_err_old_FL = K_FL.get_posRW_error(1); posRW_err_old_FR = K_FR.get_posRW_error(1); posRW_err_old_RL = K_RL.get_posRW_error(1); posRW_err_old_RR = K_RR.get_posRW_error(1);
        
        //error가 잘 나옴
        cout<< "poseRW_FL "<< posRW_FL[0] <<" ,"<< posRW_FL[1]<<endl;
        
        /****************** Conrtoller ******************/ // index [0] : R direction output, index [1] : th direction output
        FL_output[0] = C_FL.posPID(posRW_err_FL, posRW_err_old_FL, 0, 0); FL_output[1] = C_FL.posPID(posRW_err_FL, posRW_err_old_FL, 1, 0);
        //error가 안나옴.
        printf("error =%f,%f \n",posRW_err_FL[0],posRW_err_FL[1]);
        
        FR_output[0] = C_FR.posPID(posRW_err_FR, posRW_err_old_FR, 0, 1); FR_output[1] = C_FR.posPID(posRW_err_FR, posRW_err_old_FR, 1, 1);

        RL_output[0] = C_RL.posPID(posRW_err_RL, posRW_err_old_RL, 0, 2); RL_output[1] = C_RL.posPID(posRW_err_RL, posRW_err_old_RL, 1, 2);
                
        RR_output[0] = C_RR.posPID(posRW_err_RR, posRW_err_old_RR, 0, 3); RR_output[1] = C_RR.posPID(posRW_err_RR, posRW_err_old_RR, 1, 3);

        /****************** Conrtoller ******************/ // motor torque input( tau_m, tau_b)
        FL_control_input = JTrans_FL * FL_output; FR_control_input = JTrans_FR * FR_output;RL_control_input = JTrans_RL * RL_output; RR_control_input = JTrans_RR * RR_output;
        // HAA는 위치제어 걸어놔야함.
        ACT_FLHAA.Motor_taget_torque = 0 ;ACT_FRHAA.Motor_taget_torque = 0 ;ACT_RLHAA.Motor_taget_torque = 0 ;ACT_RRHAA.Motor_taget_torque = 0 ;
        ACT_FLHIP.Motor_taget_torque = FL_control_input[0]; ACT_FRHIP.Motor_taget_torque = FR_control_input[0]; ACT_RLHIP.Motor_taget_torque = RL_control_input[0]; ACT_RRHIP.Motor_taget_torque = RR_control_input[0];
        ACT_FLKNEE.Motor_taget_torque = FL_control_input[1]; ACT_FRKNEE.Motor_taget_torque = FR_control_input[1]; ACT_RLKNEE.Motor_taget_torque = RL_control_input[1]; ACT_RRKNEE.Motor_taget_torque = RR_control_input[1];


        /****************** actuator Data send to Mjdata ******************/ // d->ctrl[motor_num] = target torque
        ACT_FLHAA.DATA_Send(m,d); ACT_FLHIP.DATA_Send(m,d); ACT_FLKNEE.DATA_Send(m,d);
        ACT_FRHAA.DATA_Send(m,d); ACT_FRHIP.DATA_Send(m,d); ACT_FRKNEE.DATA_Send(m,d);
        ACT_RLHAA.DATA_Send(m,d);ACT_RLHIP.DATA_Send(m,d); ACT_RLKNEE.DATA_Send(m,d);
        ACT_RRHAA.DATA_Send(m,d);ACT_RRHIP.DATA_Send(m,d);ACT_RRKNEE.DATA_Send(m,d);
        
        //  d-> qpos[2] =0.3536;
        

        // for(int i = 0; i<13 ; i++)
        // {   
            // d-> qpos[3*i+7] = 0;
            // d-> qpos[3*i+8] = PI/4;
            // d-> qpos[3*i+9] = PI/2;
            // d->ctrl[i] = 10;
        // }


        // cout << "position FL:" << " " << d->sensordata[46] << endl;
        // cout << "position FR:" << " " << d->sensordata[49] << endl;
        // cout << "position RL:" << " " << d->sensordata[52] << endl;
        // cout << "position RR:" << " " << d->sensordata[55] << endl;
        // cout << "pose = :" << " " << 0.5-sqrt(2)*0.25 << endl;
    }

    if (loop_index % data_frequency == 0) 
    {     // loop_index를 data_frequency로 나눈 나머지가 0이면 데이터를 저장.
        save_data(m, d);
    }
    t_end  = t ;
    //loop_index += 1;
    loop_index = loop_index + 1;
}

// main function
int main(int argc, const char** argv)
{
    

    // activate software
    mj_activate("mjkey.txt");

    
    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
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
    //d-> qpos[2] =0.3536;
    fid = fopen(datafile, "w");
    init_save_data();
    for(int i = 0; i<13 ; i++)
    {   
        d-> qpos[3*i+7] = 0;
        d-> qpos[3*i+8] = PI/4;
        d-> qpos[3*i+9] = PI/2;
    }
    d-> qpos[2] = 0.6;
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


