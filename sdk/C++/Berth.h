#ifndef BERTH_H
#define BERTH_H

#include <vector>
#include <queue>
#include "constant.h"

using namespace std;

class Berth
{
public:
    int id;
    int x, y, boat_x, boat_y;
    int loading_speed; // each frame can load how many gds
    int load_num, btotal_load_num; // express current load goods num
    int boat_num; // express current ship boat num
    int r_num; // express current how many robot go to this berth
    int disToNearestT, nearestTid;
    int money;
    int current_boat_id;
    double avg_dis_to_pull;
    int total_p_bind_berth;
    static double mean_dis_to_T;
    static double mean_dis_to_gds;
    int nearHaveGoodsNum;
    queue<int> money_que;

    bool forbid;

    Berth() {
        forbid = false;
        load_num = 0;
        boat_num = 0;
        avg_dis_to_pull = 0.0;
        total_p_bind_berth = 0;
        btotal_load_num = 0;
        r_num = 0;
        disToNearestT = INT_MAX;
        nearestTid = -1;
        money = 0;
        current_boat_id = -1;
        nearHaveGoodsNum = 0;
    }

    // each frame load gds if boat at here
    pair<int, int> load(int c_capacity, int max_capacity) {
        if (load_num <= 0) return { 0, 0 };
        int gap = max_capacity - c_capacity;
        if (load_num < loading_speed) {
            int t = min(gap, load_num);
            int load_money = 0;
            int n = t;
            while(n--) {
                load_money += money_que.front();
                money_que.pop();
                --load_num;
            }
            money -= load_money;
            return { t, load_money };
        }
        else {
            int t = min(loading_speed, gap);
            int load_money = 0;
            int n = t;
            while(n--) {
                load_money += money_que.front();
                money_que.pop();
                --load_num;
            }
            money -= load_money;
            return { t, load_money };
        }
    }

    // according to x, y init robotAtxy and shipAtxy
    void initXY(char grid[][N]);

    double getAvgAddedGdnsSpeed();

    void setDisToNearestT(vector<Point>& delivery_points);

    static double getMeanDisToT();
    static double getMeanDisToGds();
    static bool judgeNeedBuyBoat();
};

#endif