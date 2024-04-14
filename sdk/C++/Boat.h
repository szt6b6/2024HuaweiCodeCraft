#ifndef BOAT_H
#define BOAT_H
#include <stdio.h>
#include "constant.h"
#include "Berth.h"

class Boat
{
public:
    static int i;

    int target_pos, status, id;
    int go_berth_id; // return to which berth

    BoatTaskType taskT;
    static int max_capacity;
    int c_capacity;
    int c_money;
    int transTime;

    bool timeout_flag;
    bool forward_flag; // berth distance - nearst berth distance > 500, go nearst berth first then go that berth

    Boat() : taskT(TO_SELL), c_capacity(0), go_berth_id(-1) {
        id = i++;
        timeout_flag = false;
        forward_flag = false;
        c_money = 0;
    }
    // 船动作更新
    void update(vector<Berth>& berth, vector<Boat>& boat, int frame);

    int findAberthToLoad(vector<Berth>& berth, vector<Boat>& boat, int frame);
};

#endif