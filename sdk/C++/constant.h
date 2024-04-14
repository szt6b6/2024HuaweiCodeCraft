#ifndef CONSTANT_H
#define CONSTANT_H
#include <vector>
#include <unordered_map>
#include <cmath>
#include <string>
#include <climits>

using namespace std;
/** 环境属性*/

const int N = 200;
const int boat_price=8000, robot_price=2000;
const int max_berth_num = 10;
const int max_robot_num = 20;
const int max_boat_num = 5;
const int max_T_num = 5;

const int flow_num = N*3+max_robot_num+2;

extern int real_max_robot_num;
extern int real_max_boat_num;
extern int robot_num;
extern int boat_num;
extern int berth_num;

extern int goods_num;
extern int total_added_goods_num;
extern int total_del_goods_num;
extern int frame_id;
extern int money;

extern int total_pull_num;
extern int total_pull_money;

extern int map_flag;

// 定义2D坐标点
class Point {
public:
    int x, y;

    Point(int x_, int y_) : x(x_), y(y_) {}
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

// 1 为陆地属性 2为海洋属性 3为二者皆可 0 为障碍物
extern unordered_map<char, int> grid_property;

// 地块障碍物通过时间
extern unordered_map<char, int> grid_time_property;

// 地块碰撞属性
extern unordered_map<char, int> collision_property;

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

// define boat task type
enum BoatAction {
    STAY = 0,
    SHIP = 1,
    ROT_0 = 2,
    ROT_1 = 3,
    RANDOM_ROT = 4,
};

#endif