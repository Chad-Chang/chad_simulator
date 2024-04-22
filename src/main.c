// #pragma once

#include "funcs.h" // 사용자 정의 헤더파일 <>하면 안됨.
void mycontroller(const mjModel* m, mjData* d) {
    mj_energyPos(m, d);
    mj_energyVel(m, d);
    /* d->energy[0] // PE
     * d->energy[1] // KE
     */
     //printf("%f %f %f %f \n", d->time, d->energy[0], d->energy[1], d->energy[0] + d->energy[1]);


     /* Check Equation
     * M*qacc + qfrc_bias = qfrc_applied + ctrl
     */
     //const int nv = 2; // DoF of system
     //double dense_M[nv * nv] = { 0 };   // 4x1 inertia matrix
    double dense_M[ndof * ndof] = { 0 };   // 4x1 inertia matrix
    mj_fullM(m, dense_M, d->qM);
    //double M[nv][nv] = { 0 };   // 2x2 inertia matrix
    double M[ndof][ndof] = { 0 };   // 2x2 inertia matrix
    M[0][0] = dense_M[0];
    M[0][1] = dense_M[1];
    M[1][0] = dense_M[2];
    M[1][1] = dense_M[3];
    /* printf("%f %f \n", M[0][0], M[0][1]);
     printf("%f %f \n", M[1][0], M[1][1]);
     printf("*********** \n");*/

    double qddot[ndof] = { 0 };
    qddot[0] = d->qacc[0];
    qddot[1] = d->qacc[1];

    double h[ndof] = { 0 };
    h[0] = d->qfrc_bias[0];
    h[1] = d->qfrc_bias[1];

    double lhs[ndof] = { 0 };
    mju_mulMatVec(lhs, dense_M, qddot, 2, 2);   // lhs = M*qddot
    lhs[0] = lhs[0] + h[0]; // lhs = M*qddot + h
    lhs[1] = lhs[1] + h[1];

    /*printf("%f %f \n", lhs[0], lhs[1]);
    printf("*********** \n");*/

    // Gravity (+ Coriolis) Compensation
    d->qfrc_applied[0] = h[0];
    d->qfrc_applied[1] = h[1];

    double rhs[ndof] = { 0 };
    rhs[0] = d->qfrc_applied[0];
    rhs[1] = d->qfrc_applied[1];

    /*printf("%f %f \n", lhs[0], rhs[0]);
    * printf("%f %f \n", lhs[1], rhs[1]);
    printf("*********** \n");*/


    // Controller
    double Kp1 = 100, Kp2 = 100;
    double Kv1 = 10, Kv2 = 10;
    double qref1 = -0.5, qref2 = -1.6;

    // PD Control
    /*d->qfrc_applied[0] = Kp1 * (qref1 - d->qpos[0]) + Kv1 * (0 - d->qvel[0]);
    d->qfrc_applied[1] = Kp2 * (qref2 - d->qpos[1]) + Kv2 * (0 - d->qvel[1]);*/

    // PD + Gravity compensation
    d->qfrc_applied[0] = h[0] + Kp1 * (qref1 - d->qpos[0]) + Kv1 * (0 - d->qvel[0]);
    d->qfrc_applied[1] = h[1] + Kp2* (qref2 - d->qpos[1]) + Kv2 * (0 - d->qvel[1]);

    // Feedbck Linearization
    // M*(PD) + h
    double tau[ndof] = { 0 };
    tau[0] = Kp1 * (qref1 - d->qpos[0]) + Kv1 * (0 - d->qvel[0]);
    tau[1] = Kp2 * (qref2 - d->qpos[1]) + Kv2 * (0 - d->qvel[1]);

    mju_mulMatVec(tau, dense_M, tau, 2, 2);
    tau[0] += h[0];
    tau[1] += h[1];

    //d->qfrc_applied[0] = tau[0];
    //d->qfrc_applied[1] = tau[1];

    d->ctrl[0] = tau[0];    // 0 : torque, 1 : pos_servo, 2 : vel_servo
    d->ctrl[3] = tau[1];    // 3 : torque, 4 : pos_servo, 5 : vel_servo

    if (loop_index % data_frequency == 0) {     // loop_index를 data_frequency로 나눈 나머지가 0이면 데이터를 저장.
        save_data(m, d);
    }
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

    fid = fopen(datapath, "w");
    init_save_data();

    d->qpos[0] = 0.1;   // joint 1 - HFE
    //d->qpos[1] = 0.5;   // joint 2 - KFE

    // use the first while condition if you want to simulate for a period.
    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0)
        {
            mj_step(m, d);

            // Drag Force = -c*v^2*unit_vector(v); v = sqrt(vx^2+vy^2+vz^2);
            // vector(v) = vx i + vy j + vz k;
            // unit vector = vector(v)/v;
            // fx = -c*v*vx; fy = -c*v*vy; fz = -c*v*vz;

            //double vx, vy, vz;
            //vx = d->qvel[0]; vy = d->qvel[1]; vz = d->qvel[2];

            //double v;
            //v = sqrt(vx^2 + vy^2 + vz^2); <-- 이렇게 하면 안되는듯 
            //v = sqrt(vx * vx + vy * vy + vz * vz);

            //double fx, fy, fz, c;
            //c = 0.5;
            //fx = -c * v * vx;
            //fy = -c * v * vy;
            //fz = -c * v * vz;

            //d->qfrc_applied[0] = fx;
            //d->qfrc_applied[1] = fy;
            //d->qfrc_applied[2] = fz;
        }

        if (d->time >= simend) {
            fclose(fid);
            break;
        }

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
