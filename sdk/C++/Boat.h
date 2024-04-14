#ifndef BOAT_H
#define BOAT_H
#include <stdio.h>
#include "constant.h"
#include "Berth.h"

class Boat
{
public:
    static double total_sold_goods_num;
    int status, id;
    int go_berth_id;
    int collide_with_id;

    BoatTaskType taskT;
    static int max_capacity;
    int c_capacity;
    int c_money;
    int x, y, dir;
    int c_p; // 当前点在路径中的位置
    BoatAction action;

    int dstx, dsty;

    vector<Point> path;

    Boat() : c_capacity(0), go_berth_id(-1) {
        c_money = 0;
        taskT = TO_LOAD;
        c_p = -1;
        action = STAY;
        collide_with_id = -1;
    }


    Boat(int startX, int startY) {
        x = startX;
        y = startY;
    }
    // 船动作更新
    void update(vector<Berth>& berth, vector<Boat>& boat, char grid[N][N], int ocean_connectField[][N], int frame);

    int findAberthToLoad(vector<Berth>& berth, vector<Boat>& boat, int ocean_connectField[][N], int frame);

    int findTtoSell(vector<Boat>& boat, int ocean_connectField[][N], int frame);

    // 根据自身方向和move方向，更新自身动作
    void moveBoat(char grid[N][N], int frame);

    // 根据自身方向获得矩形左上角和右下角坐标
    pair<Point, Point> getRect();

    pair<Point, Point> getNextRect();

    int getPriority() {
        return max_boat_num - id + (taskT == TO_SELL) * 50;
    }

    static pair<Point, Point> getRectUsePos(int p_x, int p_y, int d);
private:
    bool arrivedT();

    bool arrivedBerth(char grid[N][N]);
    
    // 获得自身旋转后的下一个点和方向
    pair<Point, int> getNextPointToRot(int rot);

    bool checkNextShipOk();

    bool checkNextRotOk(int r);

    void followPath();

    // follow路径时候无法移动或者旋转时 随机移动
    void randMoveOrRotWithSafe();

public:
static     // 返回距离最近的T的id和距离
    pair<int, int> getMinDisT2P(int x, int y);
};

#endif