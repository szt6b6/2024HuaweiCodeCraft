#ifndef TASKASSIGNER_H
#define TASKASSIGNER_H

#include <list>
#include <set>
#include "Robot.h"
#include "utils.h"
#include "constant.h"
#include "Boat.h"
#include "Berth.h"
#include "maxCostFlow.h"


using namespace std;
#define FLOAT_MIN numeric_limits<float>::min()

class TaskAssigner {
public:
    // generate task and 分配任务给机器人
    static void update(vector<Robot>& robot, vector<Berth>& berth, vector<Boat>& boat, char grid[][N], int gds[][N][2], 
                bool gds_locked[][N], int connectField[][N], int berth_dir_map[berth_num][N][N], int berth_dis_map[berth_num][N][N], int frame);
    static void updateForOneRobot(vector<Robot> &robot, vector<Berth> &berth, vector<Boat> &boat, char grid[][N], int gds[][N][2], 
                                        bool gds_locked[][N], int connectField[][N], int berth_dir_map[berth_num][N][N], 
                                        int berth_dis_map[berth_num][N][N], int frame, int r_i);

private:
    // according to position x and y, find a berth to pull
    static int findBerthToPULL(vector<Berth>& berth, vector<Boat>& boat, int berth_dis_map[berth_num][N][N], int connectField[][N], int x, int y, int frame);

    static pair<vector<int>, float> branch_and_bound();
};

#endif