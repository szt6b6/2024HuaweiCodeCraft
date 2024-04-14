#ifndef BERTH_H
#define BERTH_H

#include <vector>
#include "constant.h"

using namespace std;

class Berth
{
public:
    static int max_transport_time;
    static int min_transport_time;
    static int nearest_berth_id;
    static int farest_berth_id;
    static int average_transport_time;

    int id;
    int x, y;
    int robot_at_x, robot_at_y;
    int transport_time; // to des need frames
    int loading_speed; // each frame can load how many gds
    int load_num, btotal_load_num; // express current load goods num
    int boat_num; // express current ship boat num
    int r_num; // express current how many robot go to this berth
    double average_dis2eachPoint;
    int money;
    int current_boat_id;

    Berth() {
        load_num = 0;
        boat_num = 0;
        btotal_load_num = 0;
        r_num = 0;
        average_dis2eachPoint = 0.0;
        money = 0;
        current_boat_id = -1;
    }

    // each frame load gds if boat at here
    pair<int, int> load(int c_capacity, int max_capacity) {
        if (load_num <= 0) return { 0, 0 };
        int gap = max_capacity - c_capacity;
        if (load_num < loading_speed) {
            int t = min(gap, load_num);
            int load_money = money / load_num * t;
            load_num -= t;
            money -= load_money;
            return { t, load_money };
        }
        else {
            int t = min(loading_speed, gap);
            int load_money = money / load_num * t;
            money -= load_money;
            load_num -= t;
            return { t, load_money };
        }
    }

    // according to x, y init robotAtxy and shipAtxy
    void initXY(char grid[][N]);
};

#endif