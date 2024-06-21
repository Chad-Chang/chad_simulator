#include "funcs.h" // 사용자 정의 헤더파일 <>하면 안됨.
// namspace std
#include "actuator.h"
#include "rw_controller.h"
#include "kinematics.h"
#include "j_controller.h"

RW_Controller C_FL;RW_Controller C_FR;RW_Controller C_RL;RW_Controller C_RR;
// J_Controller JC_FLHAA;J_Controller JC_FRHAA;J_Controller JC_RLHAA;J_Controller JC_RRHAA;

// Jacobian
Matrix2d J_FL;Matrix2d J_FR;Matrix2d J_RL;Matrix2d J_RR;


Matrix2d JTrans_FL;Matrix2d JTrans_FR;Matrix2d JTrans_RL;Matrix2d JTrans_RR;

// Controller output : RW PID INPUT//
Vector2d FL_output;Vector2d FR_output;Vector2d RL_output;Vector2d RR_output;
// motor control input
Vector2d FL_control_input;Vector2d FR_control_input;Vector2d RL_control_input;Vector2d RR_control_input;
Vector2d FL_input_DOB;Vector2d FR_input_DOB;Vector2d RL_input_DOB;Vector2d RR_input_DOB;
Vector2d FL_input_DOB_old;Vector2d FR_input_DOB_old;Vector2d RL_input_DOB_old;Vector2d RR_input_DOB_old;

Vector2d QFL_input_DOB;Vector2d QFR_input_DOB;Vector2d QRL_input_DOB;Vector2d QRR_input_DOB;
Vector2d QFL_input_DOB_old;Vector2d QFR_input_DOB_old;Vector2d QRL_input_DOB_old;Vector2d QRR_input_DOB_old;

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

Vector4d r_Pgain = {40,40,40,40};
Vector4d r_Igain = {0,0,0,0};
Vector4d r_Dgain = {0,0,0,0};
Vector4d th_Pgain = {40,40,40,40};
Vector4d th_Igain = {0,0,0,0};
Vector4d th_Dgain = {0,0,0,0};

Vector2d th_acc_FL; Vector2d th_acc_FR; Vector2d th_acc_RL; Vector2d th_acc_RR;
Vector2d th_acc_FL_old; Vector2d th_acc_FR_old; Vector2d th_acc_RL_old; Vector2d th_acc_RR_old;
Vector2d th_vel_FL; Vector2d th_vel_FR; Vector2d th_vel_RL; Vector2d th_vel_RR;
Vector2d th_vel_FL_old; Vector2d th_vel_FR_old; Vector2d th_vel_RL_old; Vector2d th_vel_RR_old;



double HAA_control_input[4];
double gear_ratio = 100;
int t = 0;
int traj_t = 0;

Matrix2d inertia_jBi2BiTq_FL; Matrix2d inertia_jBi2BiTq_FR; Matrix2d inertia_jBi2BiTq_RL; Matrix2d inertia_jBi2BiTq_RR;

Matrix2d jnt2bi; 
Matrix2d bi2jnt;
extern Vector2d FL_distub; extern Vector2d FR_distub; extern Vector2d RL_distub; extern  Vector2d RR_distub;
Vector2d FL_distub_old; Vector2d FR_distub_old; Vector2d RL_distub_old; Vector2d RR_distub_old;
Vector2d QFL_distub; Vector2d QFR_distub; Vector2d QRL_distub; Vector2d QRR_distub;
// Vector2d QFL_distub_old; Vector2d QFR_distub_old; Vector2d QRL_distub_old; Vector2d QRR_distub_old;


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

        FL_input_DOB_old = FL_input_DOB; FR_input_DOB_old = FR_input_DOB;
        RL_input_DOB_old = RL_input_DOB; RR_input_DOB_old = RR_input_DOB;
        
        QFL_input_DOB_old = QFL_input_DOB; QFR_input_DOB_old = QFR_input_DOB; 
        QRL_input_DOB_old = QFL_input_DOB; QRR_input_DOB_old = QRR_input_DOB;

        FL_distub_old = FL_distub; FR_distub_old = FR_distub; RL_distub_old = RL_distub; RR_distub_old = RR_distub;

        // joint PID gain setting  => j_set_gain(Pgain, Igain, Dgain) 
        C_FL.j_set_gain(100,0,1);C_FR.j_set_gain(100,0,1);C_RL.j_set_gain(100,0,1);C_RR.j_set_gain(100,0,1); // joint controller
        C_FL.rw_set_gain(r_Pgain,r_Igain, r_Dgain, th_Pgain, th_Igain, th_Dgain);

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

        // cout<< th_vel_FL(0)  << " "<<th_vel_FR(0) << " "<<th_vel_RL(0) <<  " "<<th_vel_RR(0) << endl;
        // cout<< th_vel_FL(1)  << " "<<th_vel_FR(1) << " "<<th_vel_RL(1) <<  " "<<th_vel_RR(1) << endl;

        /****************** Kinematics ******************/
        ////////////////////// 바이아티큘러 구조일때 - real robot////////////////////////
        // K_FL.Cal_RW(ACT_FLHIP.getMotor_pos(), ACT_FLKNEE.getMotor_pos(),0); 
        // K_FR.Cal_RW(ACT_FRHIP.getMotor_pos(), ACT_FRKNEE.getMotor_pos(),1);
        // K_RL.Cal_RW(ACT_RLHIP.getMotor_pos(), ACT_RLKNEE.getMotor_pos(),2);
        // K_RR.Cal_RW(ACT_RRHIP.getMotor_pos(), ACT_RRKNEE.getMotor_pos(),3);
        
        // 파라미터 저장
        inertia_jBi2BiTq_FL = K_FL.get_RW_Jacobian_Trans()*
                            K_FL.Cal_RW_inertia(ACT_FLHIP.getMotor_pos(), ACT_FLKNEE.getMotor_pos()+ACT_FLHIP.getMotor_pos())*K_FL.get_RW_Jacobian();
        inertia_jBi2BiTq_FR = K_FR.get_RW_Jacobian_Trans()*
                            K_FR.Cal_RW_inertia(ACT_FRHIP.getMotor_pos(), ACT_FRKNEE.getMotor_pos()+ACT_FRHIP.getMotor_pos())*K_FR.get_RW_Jacobian();
        inertia_jBi2BiTq_RL = K_RL.get_RW_Jacobian_Trans()*
                            K_RL.Cal_RW_inertia(ACT_RLHIP.getMotor_pos(), ACT_RLKNEE.getMotor_pos()+ACT_RLHIP.getMotor_pos())*K_RL.get_RW_Jacobian();
        inertia_jBi2BiTq_RR = K_RR.get_RW_Jacobian_Trans()*
                            K_RR.Cal_RW_inertia(ACT_RRHIP.getMotor_pos(), ACT_RRKNEE.getMotor_pos()+ACT_RRHIP.getMotor_pos())*K_RR.get_RW_Jacobian();

        ////////////////////// 시리얼 구조일때  - mujoco ////////////////////////
        K_FL.Cal_RW(ACT_FLHIP.getMotor_pos(), ACT_FLKNEE.getMotor_pos()+ACT_FLHIP.getMotor_pos(),0); 
        K_FR.Cal_RW(ACT_FRHIP.getMotor_pos(), ACT_FRKNEE.getMotor_pos()+ACT_FRHIP.getMotor_pos(),1);
        K_RL.Cal_RW(ACT_RLHIP.getMotor_pos(), ACT_RLKNEE.getMotor_pos()+ACT_RLHIP.getMotor_pos(),2);
        K_RR.Cal_RW(ACT_RRHIP.getMotor_pos(), ACT_RRKNEE.getMotor_pos()+ACT_RRHIP.getMotor_pos(),3);
          
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
        
        
        /****************** Put the torque in Motor ******************/
        
        FL_control_input = - QFL_input_DOB + gear_ratio * JTrans_FL * FL_output; FR_control_input = - QFR_input_DOB + gear_ratio * JTrans_FR * FR_output;
        RL_control_input = - QRL_input_DOB + gear_ratio * JTrans_RL * RL_output; RR_control_input = - QRR_input_DOB + gear_ratio * JTrans_RR * RR_output;


        // 여기 disturbance 
        FL_input_DOB = - FL_distub + FL_control_input; FR_input_DOB = - FR_distub + FR_control_input;
        RL_input_DOB = - RL_distub + RL_control_input; RR_input_DOB = - RR_distub + RR_control_input;

        disturbance = 1*sin(0.001*t);
        // joint space acceleration
        th_acc_FL(0) = tustin_derivative(th_vel_FL(0),th_vel_FL_old(0),th_acc_FL_old(0),130);
        th_acc_FL(1) = tustin_derivative(th_vel_FL(1),th_vel_FL_old(1),th_acc_FL_old(1),130);

        th_acc_FR(0) = tustin_derivative(th_vel_FR(0),th_vel_FR_old(0),th_acc_FR_old(0),130);
        th_acc_FR(1) = tustin_derivative(th_vel_FR(1),th_vel_FR_old(1),th_acc_FR_old(1),130);

        th_acc_RL(0) = tustin_derivative(th_vel_RL(0),th_vel_RL_old(0),th_acc_RL_old(0),130);
        th_acc_RL(1) = tustin_derivative(th_vel_RL(1),th_vel_RL_old(1),th_acc_RL_old(1),130);

        th_acc_RR(0) = tustin_derivative(th_vel_RR(0),th_vel_RR_old(0),th_acc_RR_old(0),130);
        th_acc_RR(1) = tustin_derivative(th_vel_RR(1),th_vel_RR_old(1),th_acc_RR_old(1),130);

        // biarticular joint space acceleration
        FL_distub = inertia_jBi2BiTq_FL*jnt2bi*th_acc_FL; // biarticular torque mapping 
        FR_distub = inertia_jBi2BiTq_FR*jnt2bi*th_acc_FR;
        RL_distub = inertia_jBi2BiTq_RL*jnt2bi*th_acc_RL;
        RR_distub = inertia_jBi2BiTq_RR*jnt2bi*th_acc_RR;

        QFL_input_DOB(0) = lowpassfilter(FL_input_DOB(0),FL_input_DOB_old(0),QFL_input_DOB_old(0),120); 
        QFL_input_DOB(1) = lowpassfilter(FL_input_DOB(1),FL_input_DOB_old(1),QFL_input_DOB_old(1),120);
        
        QFR_input_DOB(0) = lowpassfilter(FR_input_DOB(0),FR_input_DOB_old(0),QFR_input_DOB_old(0),120); 
        QFR_input_DOB(1) = lowpassfilter(FR_input_DOB(1),FR_input_DOB_old(1),QFR_input_DOB_old(1),120);
        
        QRL_input_DOB(0) = lowpassfilter(RL_input_DOB(0),RL_input_DOB_old(0),QRL_input_DOB_old(0),120); 
        QRL_input_DOB(1) = lowpassfilter(RL_input_DOB(1),RL_input_DOB_old(1),QRL_input_DOB_old(1),120);

        QRR_input_DOB(0) = lowpassfilter(RR_input_DOB(0),RR_input_DOB_old(0),QRR_input_DOB_old(0),120); 
        QRR_input_DOB(1) = lowpassfilter(RR_input_DOB(1),RR_input_DOB_old(1),QRR_input_DOB_old(1),120);

        
        // QFL_distub(0) = lowpassfilter(FL_distub(0),FL_distub_old(0),QFL_distub(0),20); 
        // QFL_distub(1) = lowpassfilter(FL_distub(1),FL_distub_old(1),QFL_distub(1),20);
        
        // QFR_distub(0) = lowpassfilter(FR_distub(0),FR_distub_old(0),QFR_distub(0),20); 
        // QFR_distub(1) = lowpassfilter(FR_distub(1),FR_distub_old(1),QFR_distub(1),20);
        
        // QRL_distub(0) = lowpassfilter(RL_distub(0),RL_distub_old(0),QRL_distub(0),20); 
        // QRL_distub(1) = lowpassfilter(RL_distub(1),RL_distub_old(1),QRL_distub(1),20);

        // QRR_distub(0) = lowpassfilter(RR_distub(0),RR_distub_old(0),QRR_distub(0),20); 
        // QRR_distub(1) = lowpassfilter(RR_distub(1),RR_distub_old(1),QRR_distub(1),20);

        
        /*******************HAA position control input*******************/

        // HAA_control_input[0] = C_FL.j_posPID(PI/6,ACT_FLHAA.getMotor_pos(),T,cutoff);
        // HAA_control_input[1] = C_FR.j_posPID(PI/2,ACT_FRHAA.getMotor_pos(),T,cutoff);
        // HAA_control_input[2] = C_RL.j_posPID(PI/3,ACT_RLHAA.getMotor_pos(),T,cutoff);
        // HAA_control_input[3] = C_RR.j_posPID(PI/10,ACT_RRHAA.getMotor_pos(),T,cutoff);
        HAA_control_input[0] = C_FL.j_posPID(0,ACT_FLHAA.getMotor_pos(),T,cutoff);
        HAA_control_input[1] = C_FR.j_posPID(0,ACT_FRHAA.getMotor_pos(),T,cutoff);
        HAA_control_input[2] = C_RL.j_posPID(0,ACT_RLHAA.getMotor_pos(),T,cutoff);
        HAA_control_input[3] = C_RR.j_posPID(0,ACT_RRHAA.getMotor_pos(),T,cutoff);
        
        // d->qpos[2] =;

        FL_control_input = FL_control_input - QFL_input_DOB; FR_control_input = FR_control_input - QFR_input_DOB;
        RL_control_input = RL_control_input - QRL_input_DOB; RR_control_input = RR_control_input - QRR_input_DOB;
        
        // 입력 토크
        ACT_FLHAA.DATA_Send(d,HAA_control_input[0]);
        ACT_FLHIP.DATA_Send(d,FL_control_input[0]+FL_control_input[1]+disturbance);
        ACT_FLKNEE.DATA_Send(d,FL_control_input[1]);
        // d->ctrl[3] = 1;
        ACT_FRHAA.DATA_Send(d,HAA_control_input[1]);
        ACT_FRHIP.DATA_Send(d,FR_control_input[0]+FR_control_input[1]+disturbance);
        ACT_FRKNEE.DATA_Send(d,FR_control_input[1]);
        // d->ctrl[6] = 1;
        ACT_RLHAA.DATA_Send(d,HAA_control_input[2]);
        ACT_RLHIP.DATA_Send(d,RL_control_input[0]+RL_control_input[1]+disturbance);
        ACT_RLKNEE.DATA_Send(d,RL_control_input[1]);
        // d->ctrl[9] = 1;
        ACT_RRHAA.DATA_Send(d,HAA_control_input[3]);
        ACT_RRHIP.DATA_Send(d,RR_control_input[0]+RR_control_input[1]+disturbance);
        ACT_RRKNEE.DATA_Send(d,RR_control_input[1]);

        // ACT_FLHAA.DATA_Send(d,HAA_control_input[0]);
        // ACT_FLHIP.DATA_Send(d,FL_control_input_DOB[0]+FL_control_input_DOB[1]+disturbance);
        // ACT_FLKNEE.DATA_Send(d,FL_control_input_DOB[1]);
        // // d->ctrl[3] = 1;
        // ACT_FRHAA.DATA_Send(d,HAA_control_input[1]);
        // ACT_FRHIP.DATA_Send(d,FR_control_input_DOB[0]+FR_control_input_DOB[1]+disturbance);
        // ACT_FRKNEE.DATA_Send(d,FR_control_input_DOB[1]);
        // // d->ctrl[6] = 1;
        // ACT_RLHAA.DATA_Send(d,HAA_control_input[2]);
        // ACT_RLHIP.DATA_Send(d,RL_control_input_DOB[0]+RL_control_input_DOB[1]+disturbance);
        // ACT_RLKNEE.DATA_Send(d,RL_control_input_DOB[1]);
        // // d->ctrl[9] = 1;
        // ACT_RRHAA.DATA_Send(d,HAA_control_input[3]);
        // ACT_RRHIP.DATA_Send(d,RR_control_input_DOB[0]+RR_control_input_DOB[1]+disturbance);
        // ACT_RRKNEE.DATA_Send(d,RR_control_input_DOB[1]);

        
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
