#ifndef DATALOGGING_H_
#define DATALOGGING_H_

#include "globVariable.h"

/***************** Data Logging *****************/
FILE* fid;
int loop_index = 0;
const int data_frequency = 1; // frequency at which data is written to a file

char datapath[] = "data/data.csv";
char filename[] = "scene.xml";
char datafile[] = "data.csv";

extern Vector2d z; extern Vector2d h; extern Vector2d o;
void init_save_data()
{
    // This function is called once and is used to get the headers
    // Write name of the variable here (header)
    // comma(,) should be omitted in the last line.
    
    fprintf(fid, "t, ");
    fprintf(fid, "ref_R, ref_th, Rv, thv");
    
    
    // Don't remove the newline
    fprintf(fid, "\n");
}

void save_data(const mjModel* m, mjData* d, StateModel_* state_model)
{
    // This function is called at a set frequency,put data here.
    // Data here should correspond to headers in init_save_data()
    // Seperate data by a space %f followed by space
    // comma(,) should be omitted in the last line.
    double touch = d->sensordata[8];
    double grf_x = d->sensordata[9];
    double grf_y = d->sensordata[10];
    double grf_z = d->sensordata[11];
    double trunk_vel_y = d->sensordata[15];

    double cartesian_grf_x = grf_x * cos(pi - state_model->q_bi[1]) - grf_y * sin(pi - state_model->q_bi[1]);
    double cartesian_grf_y = grf_x * sin(pi - state_model->q_bi[1]) + grf_y * cos(pi - state_model->q_bi[1]); 

    double grf_r = cartesian_grf_y * cos(state_model->posRW[1] - pi / 2) + cartesian_grf_x * sin(state_model->posRW[1] - pi / 2);
    double grf_thetar = cartesian_grf_y * sin(state_model->posRW[1] - pi / 2) - cartesian_grf_x * cos(state_model->posRW[1] - pi / 2);

    fprintf(fid, "%f, ", d->time);
    
    // fprintf(fid, "%f, %f, %f, %f, ", state_model->posRW_ref[0], state_model->posRW_ref[1], state_model->posRW[0], state_model->posRW[1]);
    // fprintf(fid, "%f, %f",disturbance[0], -d0[0]);
    
    // Vector2d leg_rw_ref_pos = state_model->posRW_ref;
    // Vector2d leg_rw_pos = state_model->posRW;
    
    // 여기분터 
    // fprintf(fid, "%f, %f, %f", leg_rw_ref_pos[0],leg_rw_pos[0],leg_rw_ref_pos[0]-leg_rw_pos[0]);
    fprintf(fid, "%f, %f, %f, %f",z[0],z[1] ,o[0],o[1]);
    // printf("%f, %f, %f, %f \n",z[0],z[1] ,o[0],o[1]);


    
    
    // // Don't remove the newline
    fprintf(fid, "\n");


}

#endif // DATALOGGING_H_