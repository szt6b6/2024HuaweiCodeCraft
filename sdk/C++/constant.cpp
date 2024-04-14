#include "constant.h"


int real_max_robot_num = 16;
int real_max_boat_num = 2;
int robot_num = 0;
int boat_num = 0;
int berth_num = 0;

int goods_num = 0;
int total_added_goods_num = 0;
int total_del_goods_num = 0;
int frame_id = 0;
int money = 0;

int total_pull_num = 0;
int total_pull_money = 0;

int map_flag = 0;

unordered_map<char, int> grid_property = {
    {'.' , 1}, // 空地
    {'>' , 1}, // 陆地主干道
    {'*' , 2}, // 海洋
    {'~' , 2}, // 海洋主航道
    {'#' , 0}, // 障碍
    {'R' , 1}, // 机器人购买地块，同时该地块也是主干道
    {'S' , 2}, // 船舶购买地块，同时该地块也是主航道
    {'B' , 3}, // 泊位
    {'K' , 2}, // 靠泊区
    {'C' , 3}, // 海陆立体交通地块
    {'c' , 3}, // 海陆立体交通地块，同时为主干道和主航道
    {'T' , 2} // 交货点
};

unordered_map<char, int> grid_time_property = {
    {'.' , 1}, // 空地
    {'>' , 1}, // 陆地主干道
    {'*' , 1}, // 海洋
    {'~' , 2}, // 海洋主航道
    {'#' , 999}, // 障碍
    {'R' , 1}, // 机器人购买地块，同时该地块也是主干道
    {'S' , 2}, // 船舶购买地块，同时该地块也是主航道
    {'B' , 2}, // 泊位
    {'K' , 2}, // 靠泊区
    {'C' , 2}, // 海陆立体交通地块
    {'c' , 2}, // 海陆立体交通地块，同时为主干道和主航道, 对船单独设
    {'T' , 2} // 交货点
};

unordered_map<char, int> collision_property = {
    {'.' , 1}, // 空地
    {'>' , 0}, // 陆地主干道
    {'*' , 1}, // 海洋
    {'~' , 0}, // 海洋主航道
    {'#' , 1}, // 障碍
    {'R' , 0}, // 机器人购买地块，同时该地块也是主干道
    {'S' , 0}, // 船舶购买地块，同时该地块也是主航道
    {'B' , 0}, // 泊位
    {'K' , 0}, // 靠泊区
    {'C' , 1}, // 海陆立体交通地块
    {'c' , 0}, // 海陆立体交通地块，同时为主干道和主航道
    {'T' , 0} // 交货点
};