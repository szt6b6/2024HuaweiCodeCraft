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

#define limited_frame_start 15000
#define limited_frame_end 15000
#define DEBUG 0

#define see_crash 1
#define LOG_see_crash(frame, id, info) { if(see_crash && frame < limited_frame_end && frame > limited_frame_start) fprintf(stderr, #frame " %d, " #id " %d, " #info "\n", frame, id);};
#define see_log if(frame_id < limited_frame_end && frame_id > limited_frame_start)
// #define SEE_MAP

// 定义A*节点
class Node {
public:
    Point point;
    int g; // 起始点到当前点的实际代价
    int h; // 当前点到目标点的估计代价
    int f; // f = g + h
    Node(){}
    Node(const Point& point_) : point(point_), g(1), h(0), f(1) {}
    Node(const Point& point_, int g, int h) : point(point_), g(g), h(h), f(g + h) {}

    // 用于优先队列的比较函数
    bool operator>(const Node& other) const {
        return f > other.f;
    }

    Node& operator=(const Node& b) {
        point = b.point;
        g = b.g;
        h = b.h;
        f = b.f;
        return *this;
    }
};

class BFS_Node {
public:

    Point point;
    int g; // 起始点到当前点的实际代价
    int h; // 当前点到目标点的估计代价
    int f; // f = g + h
    BFS_Node(const Point& point_) : point(point_), g(1), h(0), f(1) {}
    BFS_Node(const Point& point_, int g, int h) : point(point_), g(g), h(h), f(g + h) {}

    // 用于优先队列的比较函数
    bool operator>(const BFS_Node& other) const {
        return g > other.g;
    }

    BFS_Node& operator=(const BFS_Node& b) {
        point = b.point;
        g = b.g;
        h = b.h;
        f = b.f;
        return *this;
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
    Task() : dest_bert_id(0) {}

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

// 主要目的是用来存储生成的每个商品到地图上每个地块的距离
class Good {
public:
    int* dis_map;
    int* dir_map;
    int x, y;
    Good() : dis_map(nullptr) {}

    Good(int x_, int y_) : x(x_), y(y_){
        dis_map = new int[N*N];
        dir_map = new int[N*N];
        init();
    }

    // 返回i j点离x y的距离
    int getDis(int i, int j) { return *(dis_map + i * N + j);}
    // 返回i j点到x y的方向
    int getDir(int i, int j) { return *(dir_map + i * N + j);}

    void free_m() {
        if(dis_map != nullptr) {
            delete[] dis_map;
            dis_map = nullptr;
        }
        if(dir_map != nullptr) {
            delete[] dir_map;
            dir_map = nullptr;
        }
    }

private:
    void init();
};

// A* algorithm, find a path from (sx, sy) to (dx, dy) in map
vector<Point> aStar(char grid[][N], int connectField[][N], vector<Robot>& robot, int sx, int sy, int dx, int dy, int frame, int r_id);

// bfs algorithm, find a path from (sx, sy) to (dx, dy) in map
vector<Point> bfs_path(char grid[][N], int connectField[][N], vector<Robot>& robot, int sx, int sy, int dx, int dy, int frame, int r_id);

// bfs algorithm, find a path from (sx, sy) to (dx, dy) in map
vector<Point> boat_bfs_path(char grid[][N], int connectField[][N], vector<Boat>& boat, int sx, int sy, int dx, int dy, int frame, int r_id);

// collect at which frame grid will have robot
void setFrameObMap(vector<Robot>& robot, int frame, int r_id);

// an other get path way, start is a berth, end is a point
vector<Point> getPathFromBerthDirMap(int berth_dir_map[][N][N], int mbx, int mby, int berth_x, int berth_y, int berth_id, bool rev);

// generate a random Point between [1, n]
Point randomPoint(int n, char grid[][N]);

// get the direction from current point to next point
int getDirection(int x, int y);

// find position in path, to avoid timeout
size_t findPosition(vector<Point>& path, int x, int y);

// lian tong field split
void connectFieldSplit(char grid[][N], int connectField[][N]);
void ocean_connectFieldSplit(char grid[][N], int ocean_connectField[][N]);

// get millisecond
long long getMs();

// delay s
void delay(int s);

// euler distance
double eulerDistance(int x, int y, int x1, int y1);

// 计算两点之间的曼哈顿距离
int manhattanDistance(const Point& a, const Point& b);
int manhattanDistance(int x, int y, int x1, int y1);

// 初始化陆地中的点到港口的距离和方向
void initBerthDirAndDisMap(int berth_dir_map[][N][N], int berth_dis_map[][N][N], char grid[][N], vector<Berth>& berth);

// 初始化海洋中的点到T点的距离
void initOceanTDisMap(int ocean_T_dis_map[][N][N], char grid[][N], vector<Point>& T);

// 初始化海洋中的点到港口的距离
void initOceanBerthDisMap(int ocean_berth_dis_map[][N][N], char grid[][N], vector<Berth>& berth);

// 初始化地图行走代价 主要目的是将主航道周围代价增加
void initGridCost(char grid[][N], int grid_cost[][N][N]);

// count nearby robot num
int countNearRobotNum(int x, int y, char grid[N][N], vector<Robot>& robot);

// chekc if 2 robots will happen collision
bool checkCollisionBetwRobot(vector<Robot>& robot, int r1_id, int r2_id);

// check if 2 boats will happen collision
bool checkCollisionBetwBoat(vector<Boat>& boat, int b1_id, int b2_id);

// init priority task to robot assign relationship
void init_taskBindQue(priority_queue<RobotTaskProfit, vector<RobotTaskProfit>, greater<RobotTaskProfit>>& taskBindQue, \
                    int taskIds_map[N][N], char grid[N][N], vector<Robot>& robot, vector<Task>& taskQueue, int frame);

// 针对船往不同方向走 做一下地图填充
void intiBoatDirCanGoMap(char grid[N][N], bool boat_dir_can_go_map[4][N][N]);

// 返回平均每帧场上新增货物的速度
double getAvgNewGoodsSpeed();

// 返回平均每帧所有机器人搬运货物到港口的速度
double getAvgRobotsPullSpeed();

// 返回平均每帧所有船去卖货的速度
double getAvgBoatToSellSpeed();

inline void banBerth(int frame, int berth_id){
    if(frame_id == frame) berth[berth_id].forbid = true;
}
#endif