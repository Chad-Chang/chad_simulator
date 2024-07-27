#ifndef DATALOGGING_H_
#define DATALOGGING_H_

#include "globVariable.h"

/***************** Data Logging *****************/
FILE* fid_FL;
FILE* fid_FR;
FILE* fid_RL;
FILE* fid_RR;
FILE* fid_Trunk;
extern Vector2d d_hat;
extern double disturb;

int loop_index = 0;
const int data_frequency = 100; // frequency at which data is written to a file

char datapath_FL[] = "../data/data_FL.csv";
char datapath_FR[] = "../data/data_FR.csv";
char datapath_RL[] = "../data/data_RL.csv";
char datapath_RR[] = "../data/data_RR.csv";
char datapath_Trunk[] = "../data/data_trunk.csv";

// XML File Logging
char filename[] = "../data/scene.xml";

void init_save_data_leg()
{
    // This function is called once and is used to get the headers
    // Write name of the variable here (header)
    // comma(,) should be omitted in the last line.
    
    fprintf(fid_FL, "t, ");
    fprintf(fid_FL, "dist_ref, dist_hat ");
    
    
    // Don't remove the newline
    fprintf(fid_FL, "\n");
}

void save_data_leg(const mjModel* m, mjData* d)
{
    // This function is called at a set frequency,put data here.
    // Data here should correspond to headers in init_save_data()
    // Seperate data by a space %f followed by space
    // comma(,) should be omitted in the last line.
   
    fprintf(fid_FL, "%f, ", d->time);
    fprintf(fid_FL, "%f, %f ", disturb, -d_hat[0]);
    

    // // Don't remove the newline
    fprintf(fid_FL, "\n");


}

// void init_save_data_trunk(FILE* fid)
// {
//     // This function is called once and is used to get the headers
//     // Write name of the variable here (header)
//     // comma(,) should be omitted in the last line.

//     fprintf(fid, "t,");
//     fprintf(fid, "touch_FL, touch_FR, touch_RL, touch_RR");

//     // Don't remove the newline
//     fprintf(fid, "\n");
// }

// void save_data_trunk(const mjModel* m, mjData* d, FILE* fid)
// {
//     // This function is called once and is used to get the headers
//     // Write name of the variable here (header)
//     // comma(,) should be omitted in the last line.

//     double touch_FL = d->sensordata[18];
//     double touch_FR = d->sensordata[22];
//     double touch_RL = d->sensordata[26];
//     double touch_RR = d->sensordata[30];

//     // touch sensor 넣기, GRF 측정값들 여기에 넣기
//     fprintf(fid, "%f, ", d->time);
//     fprintf(fid, "%f, %f, %f, %f ", touch_FL, touch_FR, touch_RL, touch_RR);

//     // Don't remove the newline
//     fprintf(fid, "\n");
// };

#endif // DATALOGGING_H_