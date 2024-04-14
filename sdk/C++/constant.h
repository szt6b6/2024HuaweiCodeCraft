#ifndef CONSTANT_H
#define CONSTANT_H
#include <vector>
#include <unordered_map>
#include <cmath>
#include <string>
#include <climits>

using namespace std;
/** 环境属性*/
const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 202;
const int flow_num = 10 * N + 10 + 2; // 最大流最多节点数量 机器人数+任务数 <= flow_num - 2
// const int flow_num = N*N*berth_num;
const int boat_num = 5;
const int berth2berth_dis = 500;

constexpr float float_max = numeric_limits<float>::max();


extern int total_load_num;
extern int total_pull_money;

/**----------参数区域---------------------*/
/**all express as profit, look for greater profit*/
// good bind to which berth
extern float berth_dismap_weight;
extern float berth_loading_weight;
extern float berth_averageDis_weight;
extern float berth_haveBoatNum_weight;
extern float berth_haveRnum_weight;

// robot pick which task
extern float robot_moneyDis_weight;
extern float robot_avgDis_weight;
extern float robot_haveRnum_weight;
extern float robot_haveBoat_weight;

// boat go to which berth
extern float boat_loadNumAndTime_weight;
extern float boat_robotNum_weight ;
extern float boat_avgDis_weight;

// utils.cpp 
extern float gap_frame_weight;
/** ------------------------------------- */

// 定义2D坐标点
class Point {
public:
    int x, y;

    Point(int x, int y) : x(x), y(y) {}
    Point() : x(0), y(0) {}

    bool operator==(const Point& b) {
        return x == b.x && y == b.y;
    }

    const Point& operator=(const Point& b) {
        x = b.x;
        y = b.y;
        return *this;
    }
};

// 定义移动方向
const vector<Point> directions = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}}; // 0right 1left 2up 3down, next_p - current_p = direction
const char directions_s[4][10] = {"right", "left", "up", "down"};

// define robot task type
enum RobotTaskType {
    FREE = 0,
    GET = 1,
    PULL = 2
};

// define boat task type
enum BoatTaskType {
    LOADING = 0,
    TO_SELL = 1,
    TO_LOAD = 2,
};

/* map data
    . : empty land
    * : ocean
    # : obstacle
    A : robot start point
    B : berth position 4x4

    机器人占4个格子 似乎左下角是其坐标格 需要对地图做扩充处理 障碍物都往左下分别扩充一格

    - each frame search path using A* will get timeout

    - robot can GET/PULL and go at one frame
*/

/*
    path conflict
    robot {
        robot avoid state: 0/normal 1/avoid // avoid state 
        robot task type: 0/none 1/GET 2/PULL // task type
    }

    sz


*/
#endif