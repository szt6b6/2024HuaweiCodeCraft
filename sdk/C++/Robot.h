#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <stdio.h>
#include "constant.h"
#include "Berth.h"
#include "Boat.h"

using namespace std;

class Robot
{
public:
    int id;
    int x, y, good_nums;
    int move;
    bool is_avoid;
    RobotTaskType taskT;
    int mbx, mby, msx, msy;
    int dst_pull_berth_id; // when assign task, means to which berth to pull gds
    int after_pull_at_berth_id; // after pull, record current at berth_id
    vector<Point> path; // path to go, front is next point
    int get_deadline; // get goods deadline
    int money;
    int type;

    double priority;
    Robot() {
        type = 0;
        taskT = FREE;
        is_avoid = false;
        after_pull_at_berth_id = -1;
        money = 0;
        priority = id;
        move = -1;
    }

    bool arriveGetDst() {
        return x == mbx && y == mby;
    }

    bool arrivePullDst(char grid[][N]);

    void update(char grid[][N], int gds[][N][2], int connectField[][N], 
                                int berth_dir_map[][N][N], int berth_dis_map[][N][N],
                                vector<Robot>& robot, vector<Berth>& berth, vector<Boat>& boat, int frame);
    
    void setBSxy(int bx, int by, int sx, int sy);

    void moveWithNoRobotFront(char grid[][N], vector<Robot>& robot);

    // check robot around with distance d weather exist other robot
    bool checkAroundHaveRobot(char grid[][N], vector<Robot>& robot, int d);

    int gatherAroundRobotNum(vector<Robot>& robot, int d);

    Point getNextPoint();
};  


#endif