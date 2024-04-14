#include "constant.h"


float berth_dismap_weight = -1.0;
float berth_loading_weight = +0.0;
float berth_averageDis_weight = -0.0;
float berth_haveBoatNum_weight = -0.0;
float berth_haveRnum_weight = -0.0;

float robot_moneyDis_weight = 1.0;
float robot_avgDis_weight = -0.0;
float robot_haveRnum_weight = +0.0;
float robot_haveBoat_weight = +0.0;

float boat_loadNumAndTime_weight = 1.0;
float boat_robotNum_weight = -0.0;
float boat_avgDis_weight = -0.00;

float gap_frame_weight = 0.058;

int total_load_num = 0;
int total_pull_money = 0;