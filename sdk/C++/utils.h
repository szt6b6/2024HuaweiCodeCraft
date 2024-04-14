#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <queue>
#include <thread>
#include "constant.h"
#include "Berth.h"
#include "Robot.h"

using namespace std;

extern vector<Berth> berth;

#define limited_frame_start 0
#define limited_frame_end 15000
#define DEBUG 0

#define see_position 0 // open err log or not
#define LOG_see_position(frame, i, x, y, x1, y1) { if(see_position && frame < limited_frame_end && frame > limited_frame_start) fprintf(stderr, #frame " %d, robot %d, at (%d,%d), to dst (%d,%d)\n", frame, i, x, y, x1, y1);};
#define see_info 0
#define LOG_INFO(frame, id, info) { if(see_info && frame < limited_frame_end && frame > limited_frame_start) fprintf(stderr, #frame " %d, " #id " %d, " #info "\n", frame, id);};
#define see_boat 1
#define LOG_see_boat(frame, id, taskT, from, to, c_capacity, status, transTime) { if(see_boat && frame < limited_frame_end && frame > limited_frame_start) fprintf(stderr, #frame " %d, boat %d, taskT %d, from %d, to %d, c_capacity %d, status %d, transTime %d\n", frame, id, taskT, from, to, c_capacity, status, transTime);};
#define see_rmove 0
#define LOG_see_rmove(frame, id, info) { if(see_rmove && frame < limited_frame_end && frame > limited_frame_start) fprintf(stderr, #frame " %d, " #id " %d, "  "move %s\n", frame, id, info);};
#define see_robot 0
#define LOG_see_robot(frame, id, taskT, good_money, target_berth, get_x, get_y, pull_x, pull_y, x, y) { \
        if(see_robot && frame < limited_frame_end && frame > limited_frame_start) \
            fprintf(stderr, #frame " %d, " #id " %d, taskT " " %d, goodMoney " " %d, target berth %d, get_xy (%d, %d), pull_xy (%d, %d), c_xy (%d, %d)\n", \
                        frame, id, taskT, good_money, target_berth, get_x, get_y, pull_x, pull_y, x, y); \
        };
#define see_berth 1
#define LOG_see_berth(frame, id, boat_num, load_num, money) { if(see_berth && frame < limited_frame_end && frame > limited_frame_start) fprintf(stderr, #frame " %d, berth %d, boat_num %d, load_num %d, money %d, to_v dis: %d\n", frame, id, boat_num, load_num, money, berth[id].transport_time);};
#define see_task 1
#define LOG_see_task(frame, id, task_num) { if(see_task && frame < limited_frame_end && frame > limited_frame_start) fprintf(stderr, #frame " %d, robot_num: %d, task_num: %d\n", frame, id, task_num);};
#define see_crash 1
#define LOG_see_crash(frame, id, info) { if(see_crash && frame < limited_frame_end && frame > limited_frame_start) fprintf(stderr, #frame " %d, " #id " %d, " #info "\n", frame, id);};

// #define SEE_MAP

// 定义A*节点
struct Node {
    Point point;
    int g; // 起始点到当前点的实际代价
    int h; // 当前点到目标点的估计代价
    int f; // f = g + h
    Node(const Point& point) : point(point), g(1), h(0), f(1) {}
    Node(const Point& point, int g, int h) : point(point), g(g), h(h), f(g + h) {}

    // 用于优先队列的比较函数
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};
class Task {
public:
    // at which frame, good disappear
    int deadline;
    // dest_GET, dest_PULL
    int b_x, b_y, s_x, s_y, money;

    // dest_bert_id
    int dest_bert_id;
    int to_dest_berth_dis;
    Task(){}

    Task(int deadline, int b_x, int b_y, int s_x, int s_y, int money, int berth_i, int dis) :
        deadline(deadline), b_x(b_x), b_y(b_y), s_x(s_x), s_y(s_y), money(money), dest_bert_id(berth_i), to_dest_berth_dis(dis){}


    const Task& operator=(const Task& other) {
        if (this != &other) {
            deadline = other.deadline;
            b_x = other.b_x;
            b_y = other.b_y;
            s_x = other.s_x;
            s_y = other.s_y;
            money = other.money;
            dest_bert_id = other.dest_bert_id;
            to_dest_berth_dis = other.to_dest_berth_dis;
        }
        return *this;
    }
};

class RobotTaskProfit {
public:
    int r_id;
    int t_id;
    double profit;
    RobotTaskProfit(int r_id, int t_id, double profit) : r_id(r_id), t_id(t_id), profit(profit) {}

    // 用于优先队列的比较函数
    bool operator>(const RobotTaskProfit& other) const {
        return profit < other.profit;
    }
};

// A* algorithm, find a path from (sx, sy) to (dx, dy) in map
vector<Point> aStar(char grid[][N], int connectField[][N], vector<Robot>& robot, int sx, int sy, int dx, int dy, int frame, int r_id);

// bfs algorithm, find a path from (sx, sy) to (dx, dy) in map
vector<Point> bfs_path(char grid[][N], int connectField[][N], vector<Robot>& robot, int sx, int sy, int dx, int dy, int frame, int r_id);

// collect at which frame grid will have robot
void setFrameObMap(vector<Robot>& robot, int frame, int r_id);

// an other get path way, start is a berth, end is a point
vector<Point> getPathFromBerthDirMap(int berth_dir_map[berth_num][N][N], int mbx, int mby, int berth_x, int berth_y, int berth_id);

// generate a random Point between [1, n]
Point randomPoint(int n, char grid[][N]);

// get the direction from current point to next point
int getDirection(int x, int y);

// find position in path, to avoid timeout
int findPosition(vector<Point>& path, int x, int y);

// expand obstacle to left and down
void obstacle_expand(char grid[][N]);

// lian tong field split
void connectFieldSplit(char grid[][N], int connectField[][N]);

// get millisecond
long long getMs();

// delay s
void delay(int s);

// euler distance
double eulerDistance(int x, int y, int x1, int y1);

// 计算两点之间的曼哈顿距离
int manhattanDistance(const Point& a, const Point& b);
int manhattanDistance(int x, int y, int x1, int y1);

// gather road direction about all point in map to a specific berth
void initBerthDirAndDisMap(int berth_dir_map[berth_num][N][N], int berth_dis_map[berth_num][N][N], char grid[][N], vector<Berth>& berth);

// count nearby ocean gird num of a grid
// if return >= 2, means it is in ocean side
int countOceanGrid(int x, int y, char grid[][N]);

// count nearby obstacle num
int countObstacleGrid(int x, int y, char grid[N][N]);

// count nearby robot num
int countNearRobotNum(int x, int y, char grid[N][N], vector<Robot>& robot);

// judge a robot around if exist path so other robot can pass it
bool judgeRobotAroundIfPass(int x, int y, char grid[N][N]);

// fill given aren to ocean
void fillOcean(char grid[][N], int x1, int y1, int x2, int y2);

// chekc if 2 robots will happen collision
bool checkCollisionBetwRobot(vector<Robot>& robot, int r1_id, int r2_id);

// 初始化非pull状态机器人到所有可达点的距离
void init_robots2eachGDis(int robot_dis_map[robot_num][N][N], char grid[N][N], vector<Robot>& robot);
#endif