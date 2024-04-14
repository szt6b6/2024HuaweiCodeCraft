#include "Robot.h"
#include "Berth.h"
#include "utils.h"

extern int ocean_connectField[N][N];
extern int T_ocean_dis_map[max_T_num][N][N];
extern vector<Berth> berth;


double Berth::mean_dis_to_gds = -1.0;
double Berth::mean_dis_to_T = -1.0;
// 将船过来的目的点设成附近的K
void Berth::initXY(char grid[][N])
{
    // 首先找到那个方向是陆地
    // int i = 0;
    // for(i=0; i<4; i++) {
    //     Point p = { x + 2*directions[i].x, y + 2*directions[i].y };
    //     if(grid[p.x][p.y] == '.') break;
    // }
    // int k_dir = -1;
    // if(i == 0) k_dir = 1;
    // else if(i == 1) k_dir = 0;
    // else if(i == 2) k_dir = 3;
    // else if(i == 3) k_dir = 2;

    // boat_x = x, boat_y = y;
    // while(grid[boat_x][boat_y] != 'K') {
    //     boat_x += directions[k_dir].x;
    //     boat_y += directions[k_dir].y;
    // }

    boat_x = x, boat_y = y;
    
    // fprintf(stderr, "Berth %d initXY %d %d\n", id, boat_x, boat_y);
}

double Berth::getAvgAddedGdnsSpeed()
{
    return btotal_load_num / (frame_id + 0.01);
}

void Berth::setDisToNearestT(vector<Point> &delivery_points)
{
    for(int i=0; i<delivery_points.size(); i++){
        if(ocean_connectField[x][y] != ocean_connectField[delivery_points[i].x][delivery_points[i].y]) continue;
        if(disToNearestT > T_ocean_dis_map[i][x][y]) {
            disToNearestT = T_ocean_dis_map[i][x][y];
            nearestTid = i;
        }
    }
}

double Berth::getMeanDisToT()
{
    if(mean_dis_to_T != -1.0) return mean_dis_to_T;
    for(int i=0; i<berth_num; i++) {
        mean_dis_to_T += berth[i].disToNearestT;
    }
    mean_dis_to_T /= berth_num;
    return mean_dis_to_T;
}

double Berth::getMeanDisToGds()
{
    if(mean_dis_to_gds != -1.0) return mean_dis_to_gds;
    for(int i=0; i<berth_num; i++) {
        mean_dis_to_gds += berth[i].avg_dis_to_pull;
    }
    mean_dis_to_gds /= berth_num;
    return mean_dis_to_gds;
}

bool Berth::judgeNeedBuyBoat()
{
    // extern int boat_num;
    // fprintf(stderr, "values: meanDisToGds %.3f, cost %.3f, meanDisToT %.3f, cost %.3f\n", \
    //                 getMeanDisToGds(), (getMeanDisToGds() / (0.01 + max_robot_num)), \
    //                 getMeanDisToT(), (getMeanDisToT() / (0.01 + boat_num * Boat::max_capacity)));
    // // 机器人平均几帧送一个商品到港口, 船平均几帧卖掉一个商品
    // return (getMeanDisToGds() / (0.01 + max_robot_num)) < 2.0*(getMeanDisToT() / (0.01 + boat_num * Boat::max_capacity));

    return getMeanDisToGds() * 2.5 < getMeanDisToT();
}
