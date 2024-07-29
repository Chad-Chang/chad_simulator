#include "funcs.h" // 사용자 정의 헤더파일 <>하면 안됨.
// namspace std
#include "actuator.h"
#include "rw_controller.h"
#include "kinematics.h"
#include "j_controller.h"
#include "trajectory.h"
// #include ""
double Ts =0.0001;

Vector2d DOB_data;
Vector2d lowpassfilter2(Vector2d input, Vector2d input_old, Vector2d output_old, double cutoff_freq)
{
    //double cutoff_freq = 100;
    double time_const = 1 / (2 * pi * cutoff_freq);
    Vector2d output ;
    output <<0, 0;

    output = (Ts * (input + input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}
Vector2d tustin_derivative2(Vector2d input, Vector2d input_old, Vector2d output_old, double cutoff_freq)
{
    double time_const = 1 / (2 * pi * cutoff_freq);
    Vector2d output;
    output << 0,0;

    output = (2 * (input - input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}

Matrix2d cal_param(Matrix2d jacbRW,Matrix2d jacbRW_trans,Vector2d q_bi)
{
    /* Trunk Parameters */
    double m_hip = 2.5;
    double m_trunk_front = 10.;
    double m_trunk_rear = 18.;
    double m_trunk = 4 * m_hip + m_trunk_front + m_trunk_rear;

    /* Leg Parameters */
    double L = 0.25;
    double d_thigh = 0.11017; // local position of CoM of thigh
    double d_shank = 0.12997; // local position of CoM of shank
    // printf("d_thigh : %f, d_shank : %f \n", d_thigh, d_shank);

    double m_thigh = 1.017; // mass of thigh link
    double m_shank = 0.143; // mass of shank link
    double m_leg = m_thigh + m_shank;
    double m_total = m_trunk + 4 * m_leg;
    // printf("m_thigh : %f, m_shank : %f \n", m_thigh, m_shank);

    double Izz_thigh = 0.0057;     // MoI of thigh w.r.t. CoM
    double Izz_shank = 8.0318e-04; // MoI of shank w.r.t. CoM
    // printf("Izz_thigh : %f, Izz_shank : %f \n", Izz_thigh, Izz_shank);

    double Jzz_thigh =
        Izz_thigh + m_thigh * pow(d_thigh, 2); // MoI of thigh w.r.t. HFE
    double Jzz_shank =
        Izz_shank + m_shank * pow(d_shank, 2); // MoI of thigh w.r.t. KFE
    // printf("Jzz_thigh : %f, Jzz_shank : %f \n", Jzz_thigh, Jzz_shank);

    double M1 = Jzz_thigh + m_shank * pow(L, 2);
    double M2 = m_shank * d_shank * L * cos(q_bi[1]);
    double M12 = Jzz_shank;

    double JzzR_thigh  = Jzz_thigh + Jzz_shank + m_shank * pow(L, 2) - 2 * m_shank * d_shank * L * cos(q_bi[1]);
    double JzzR_couple = Jzz_thigh + m_shank * pow(L, 2) - Jzz_shank;
    double JzzR_shank = Jzz_thigh + Jzz_shank+ m_shank * pow(L, 2) + 2 * m_shank * d_shank * L * cos(q_bi[1]);

    Matrix2d MatInertia_RW;
    MatInertia_RW(0,0) = JzzR_thigh / (4 * pow(L, 2) * pow(sin(q_bi[1] / 2), 2));
    MatInertia_RW(0,1) = JzzR_couple / (2 * pow(L, 2) * sin(q_bi[1]));
    MatInertia_RW(1,0) = JzzR_couple / (2 * pow(L, 2) * sin(q_bi[1]));
    MatInertia_RW(1,1) = JzzR_shank / (4 * pow(L, 2) * pow(cos(q_bi[1] / 2), 2));
        
    
    Matrix2d Inertia_DOB;
    Inertia_DOB(0,0) = MatInertia_RW(0,0);
    Inertia_DOB(0,1) = 0;
    Inertia_DOB(1,0) = 0;
    Inertia_DOB(1,1) = MatInertia_RW(1,1);

    
    Matrix2d Lamda_nominal_DOB = jacbRW_trans*Inertia_DOB*jacbRW;

    return Lamda_nominal_DOB;
}


mjvFigure figPosRW;         // RW position tracking plot
mjvFigure figFOB;           // RWFOB GRF estimation plot
mjvFigure figTrunkState;    // Trunk state plot

double simEndtime = 100;	// Simulation End Time

double disturb;

Vector2d leg_pid_output;
Matrix2d lhs;
Matrix2d Qlhs;
Matrix2d rhs;
Matrix2d Qrhs;
Vector2d d_hat;
Vector2d ctrl_input;
Vector2d Qd_hat;

Matrix2d Lambda;

Vector2d posRW_ref ; 
Matrix2d pos_err;
Matrix2d pos_err_tus;
Vector2d posRW ; 

Vector2d motor_pos;

Matrix2d motor_pos_bi;
Matrix2d motor_vel_bi;
Matrix2d motor_acc_bi;

Matrix2d RWJ;
Matrix2d RWJ_T;


void mycontroller(const mjModel* m, mjData* d)  // 제어주기 0.000025임
{   
 // d->qpos[0] = 0;
    // d->qpos[1] = 0;
    d->qpos[2] =  0.5;
    d->qpos[3] = 0.7071;
    d->qpos[4] = -0.7071;
    double L  = 0.25;
    double time_run = d->time;
    disturb = 20*sin(d->time *10);


    /* Trajectory Generation */
    // time_run = 0;
    posRW_ref << 0.3536 + 0.1*sin(time_run*0), pi/2;
    motor_pos << d->qpos[8] , d->qpos[9];


    motor_pos_bi.col(0) << motor_pos[0], motor_pos[0]+motor_pos[1];
    motor_vel_bi.col(0) = tustin_derivative2(motor_pos_bi.col(0), motor_pos_bi.col(1),motor_vel_bi.col(1),300);
    motor_acc_bi.col(0) = tustin_derivative2(motor_vel_bi.col(0), motor_vel_bi.col(1),motor_acc_bi.col(1),cutoff);
    

    RWJ(0, 0) = sin(motor_pos[1]/ 2);
    RWJ(0, 1) = -sin(motor_pos[1] / 2);
    RWJ(1, 0) = cos(motor_pos[1] / 2);
    RWJ(1, 1) = cos(motor_pos[1]/2);
    RWJ = L * RWJ;
    RWJ_T = RWJ.transpose();

    posRW[0] = 2 * L * cos((motor_pos[1]) / 2);
    posRW[1] = 0.5 * (motor_pos_bi.col(0)(0)+motor_pos_bi.col(0)(1));
    pos_err.col(0) = posRW_ref - posRW;

    Lambda = cal_param(RWJ, RWJ_T,motor_pos_bi.col(0));
 
    d->ctrl[0] = 5000*(0-d->qpos[7]); //FLHAA   
    d->ctrl[1] = ctrl_input[0]+ctrl_input[1] +disturb;
    d->ctrl[2] = ctrl_input[1];

    pos_err_tus.col(0) = tustin_derivative2(pos_err.col(0),pos_err.col(1),pos_err_tus.col(1),cutoff);
    leg_pid_output = RWJ_T*(1000*pos_err.col(0) + 100*pos_err_tus.col(0));

    lhs.col(0) = ctrl_input;
    Qlhs.col(0)= lowpassfilter2(lhs.col(0), lhs.col(1), Qlhs.col(1),cutoff);
    
    rhs.col(0) = Lambda * motor_acc_bi.col(0);
    
    Qrhs.col(0)= lowpassfilter2(rhs.col(0), rhs.col(1), Qrhs.col(1),cutoff);

    d_hat = Qlhs.col(0) - rhs.col(0);
    Qd_hat = Qlhs.col(0) - Qrhs.col(0);
    cout << d_hat[0]<<endl;
    DOB_data  = -d_hat;
    // d_hat = Qd_hat;
    ctrl_input = leg_pid_output + d_hat; 

    rhs.col(1) = rhs.col(0);
    lhs.col(1) = lhs.col(0);
    Qlhs.col(1) = Qlhs.col(0);
    Qrhs.col(1) = Qrhs.col(0);
    pos_err.col(1) = pos_err.col(0);
    motor_pos_bi.col(1) = motor_pos_bi.col(0);
    motor_vel_bi.col(1) = motor_vel_bi.col(0);
    motor_acc_bi.col(1) = motor_acc_bi.col(0);
    pos_err_tus.col(1) = pos_err_tus.col(0) ;
    pos_err.col(1) = pos_err.col(0) ;

    if (loop_index % data_frequency == 0) {     // loop_index를 data_frequency로 나눈 나머지가 0이면 데이터를 저장.
        save_data(m, d);
        
    }
    loop_index = loop_index + 1;
}

// main function
int main(int argc, const char** argv)
{   
    

    
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
    // init_save_data();

// 초기 각도 입력    0.546812);Actuator ACT_RLKNEE(3, 2.59478)
    
 
    
    d->qpos[0] = 0;
    d->qpos[1] = 0;
    d->qpos[2] =  0.5;   // qpos[0,1,2] : trunk pos                                                                                                                 
    //                     // qpos[3,4,5.6] : trunk orientation quaternian
    
    d->qpos[3] = 0.7041;
    d->qpos[4] = -0.7041;
    d->qpos[5] = 0;
    d->qpos[6] = 0;

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
