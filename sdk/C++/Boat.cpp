#include "Boat.h"
#include "utils.h"
#include <climits>

int Boat::max_capacity = 0;
double Boat::total_sold_goods_num = 0;

extern vector<Point> delivery_point;
extern vector<Boat> boat;
extern char grid[N][N];
extern int berth_ocean_dis_map[max_berth_num][N][N];
extern int T_ocean_dis_map[max_T_num][N][N];

void Boat::update(vector<Berth>& berth, vector<Boat>& boat, char grid[N][N], int ocean_connectField[][N], int frame)
{
	// 恢复状态
	if(status == 1) return;

    action = STAY;
	switch(taskT)
	{
		case TO_SELL:
		// 去虚拟点卖货
        {
        TO_SELL_:
            // 如果到达T点
            if(arrivedT()) {
                c_money = 0;
                c_capacity = 0;
                see_log fprintf(stderr, "frame %d, Boat %d sold goods done, total pull money %d, berths load_nums: ", frame, id, total_pull_money);
                for(int i=0; i<berth_num; i++) see_log fprintf(stderr, " %d ", berth[i].load_num);
                see_log fprintf(stderr, " total_pull nums: %d, total money %d\n", total_pull_num, money);
                go_berth_id = findAberthToLoad(berth, boat, ocean_connectField, frame);
                if(go_berth_id != -1) {
                    dstx = berth[go_berth_id].boat_x;
                    dsty = berth[go_berth_id].boat_y;
                    berth[go_berth_id].boat_num++;
                    taskT = TO_LOAD;
                    goto TO_LOAD_;
                }
            }

            c_p = findPosition(path, x, y);

            if (c_p > -1 && c_p < path.size() - 1) {
                followPath();
            }
            else {
                path = boat_bfs_path(grid, ocean_connectField, boat, x, y, dstx, dsty, frame, id);
                if(path.size() == 0) {
                    action = RANDOM_ROT;
                    return;
                }
                c_p = findPosition(path, x, y);
                if (c_p > -1 && c_p < path.size() - 1) {
                    followPath();
                }
            }
			break;
        }
            
		case TO_LOAD:
			// 虚拟点/港口去另外一个港口装货
            {
        TO_LOAD_:
                if (go_berth_id == -1) { // 一开始的时候是这种情况
                    go_berth_id = findAberthToLoad(berth, boat, ocean_connectField, frame);
                    if (go_berth_id != -1) {
                        berth[go_berth_id].boat_num++;
                        dstx = berth[go_berth_id].boat_x;
                        dsty = berth[go_berth_id].boat_y;
                    }
                }
                else if(arrivedBerth(grid)) {
                    taskT = LOADING;
                    printf("berth %d\n", id);
                    path.clear();
                    return;
                }
                c_p = findPosition(path, x, y);

                if (c_p > -1 && c_p < path.size() - 1) {
                    followPath();
                }
                else {
                    path = boat_bfs_path(grid, ocean_connectField, boat, x, y, dstx, dsty, frame, id);
                    if(path.size() == 0) {
                        action = RANDOM_ROT;
                        return;
                    }
                    c_p = findPosition(path, x, y);
                    if (c_p > -1 && c_p < path.size() - 1) {
                        followPath();
                    }
                }
                break;
            }
		case LOADING:
        {
LOADING_:
            if(frame > 14000) {
                auto [t_id, dis] = getMinDisT2P(x, y);
                if(t_id != -1 && dis + frame >= 15000-6) {
                    dstx = delivery_point[t_id].x;
                    dsty = delivery_point[t_id].y;
                    berth[go_berth_id].boat_num--;
                    go_berth_id = -1;
                    taskT = TO_SELL;
                    path.clear();
                    see_log fprintf(stderr, "frame %d, boat %d time not enough and to sell with money %d, capacity %d, total money %d\n", frame, id, c_money, c_capacity, money);
                    if(status==2) { 
                        printf("dept %d\n", id);return;
                    }
                    else {
                        goto TO_SELL_;
                    }
                }
            }

            if(status != 2) {
                printf("berth %d\n", id);
            } else {
                // 装载中
                if(c_capacity >= max_capacity) {
                    int toTid = findTtoSell(boat, ocean_connectField, frame);
                    if(toTid != -1) {
                        total_sold_goods_num += c_capacity;
                        see_log fprintf(stderr, "frame %d, boat %d full and to sell with money %d, capacity %d, current berth %d have %d num left, total money %d\n", frame, id, c_money, c_capacity, go_berth_id, berth[go_berth_id].load_num, money);
                        dstx = delivery_point[toTid].x;
                        dsty = delivery_point[toTid].y;
                        berth[go_berth_id].boat_num--;
                        go_berth_id = -1;
                        printf("dept %d\n", id);
                        taskT = TO_SELL;
                        path.clear();
                        return;
                    }
                } else if(berth[go_berth_id].load_num <= 0) {

                    // if(robot_num < real_max_robot_num && c_money+money > 4000) { // 尽量提前买全机器人
                    //     auto [nearst_T_id, dis] = getMinDisT2P(x, y);
                    //     total_sold_goods_num += c_capacity;
                    //     taskT = TO_SELL;
                    //     berth[go_berth_id].boat_num--;
                    //     go_berth_id = -1;
                    //     path.clear();
                    //     dstx = delivery_point[nearst_T_id].x;
                    //     dsty = delivery_point[nearst_T_id].y;
                    //     printf("dept %d\n", id);
                    //     return;
                    // }
                    
                    int transToBerthId = -1;
                    double e_profit = 0.0;
                    for(int i=0; i<berth_num; i++) {
                        if(berth[i].boat_num > 0 || i == go_berth_id) continue;
                        if(berth[i].nearestTid == -1) continue;
                        if(ocean_connectField[x][y] != ocean_connectField[berth[i].x][berth[i].y]) continue;
                        
                        int to_T_id = berth[i].nearestTid;
                        if(frame > 14000 && frame + berth_ocean_dis_map[i][x][y] + berth[i].disToNearestT >= 15000-10) continue;

                        int estimate_gdns = berth[i].load_num + berth[i].r_num;

                        double berth_i_avg_money = berth[i].money * 1.0 / berth[i].load_num;
                        double profit_c = min(max_capacity, c_capacity + estimate_gdns) * 1.0 * berth_i_avg_money / (berth_ocean_dis_map[i][x][y] + berth[i].disToNearestT + Berth::getMeanDisToT());

                        if(profit_c < (c_money * 1.0 / (berth[go_berth_id].disToNearestT + Berth::getMeanDisToT()))) continue;
                        
                        double profit = min(max_capacity - c_capacity, estimate_gdns) * 1.0 * berth[i].money / (berth[i].load_num + 0.01) / (berth_ocean_dis_map[i][x][y] + 0.01);
                        if(e_profit < profit) {
                            transToBerthId = i;
                            e_profit = profit;
                        }
                    }

                    if(transToBerthId != -1) {
                        berth[go_berth_id].boat_num--;
                        berth[transToBerthId].boat_num++;
                        go_berth_id = transToBerthId;
                        dstx = berth[go_berth_id].boat_x;
                        dsty = berth[go_berth_id].boat_y;
                        path.clear();
                        taskT = TO_LOAD;
                        printf("dept %d\n", id);
                        see_log fprintf(stderr, "frame %d, boat %d, current load num %d, current money %d, go to another berth %d with load_num %d to load, total money %d\n",\
                                        frame, id, c_capacity, c_money, go_berth_id, berth[go_berth_id].load_num, money);
                    } else {
                        if(go_berth_id == -1) return;

                        if(frame > 14000) {
                            int dis = berth[go_berth_id].disToNearestT;
                            if(dis + 2 * Berth::getMeanDisToT() + frame >= 15000) return;
                        }

                        // if(c_capacity < max_capacity * 0.2) return;

                        total_sold_goods_num += c_capacity;
                        taskT = TO_SELL;
                        berth[go_berth_id].boat_num--;
                        path.clear();
                        dstx = delivery_point[berth[go_berth_id].nearestTid].x;
                        dsty = delivery_point[berth[go_berth_id].nearestTid].y;
                        go_berth_id = -1;
                        printf("dept %d\n", id);
                        see_log fprintf(stderr, "boat %d, current load num %d, current money %d, go to sell directly, total money %d\n",\
                                        id, c_capacity, c_money, money);
                    }
                }
                else {
                    berth[go_berth_id].current_boat_id = id;
                    auto [n, m] = berth[go_berth_id].load(c_capacity, max_capacity);
                    c_capacity += n;
                    c_money += m;
                }
            }
            
            break;
        }
	}
}

int Boat::findTtoSell(vector<Boat>& boat, int ocean_connectField[][N], int frame) {
    // find nearest T to sell
    int T = -1, dis = INT_MAX;
    for(int i=0; i<delivery_point.size(); i++) {
        if(ocean_connectField[x][y] != ocean_connectField[delivery_point[i].x][delivery_point[i].y]) continue;
        if(dis > T_ocean_dis_map[i][x][y]) { dis = T_ocean_dis_map[i][x][y]; T = i;}
    }
    return T;
}

int Boat::findAberthToLoad(vector<Berth>& berth, vector<Boat>& boat, int ocean_connectField[][N], int frame)
{
    double e_profit = double(INT_MIN);
	int target = -1;
	for (int i = 0; i < berth_num; i++) {
        
        if(berth[i].boat_num > 0 || berth[i].forbid) continue;
        auto [nearest_T_id, dis] = getMinDisT2P(berth[i].x, berth[i].y);
		if(ocean_connectField[x][y] != ocean_connectField[berth[i].x][berth[i].y]) continue;
        
        int estimate_gdns = berth[i].load_num + berth[i].r_num;
        
        // double profit = min(max_capacity - c_capacity, berth[i].load_num) * (1.0 * berth[i].money / (berth[i].load_num + 0.01)) / (berth_ocean_dis_map[i][x][y] + 0.01);
        double profit = min(max_capacity - c_capacity, estimate_gdns) * (1.0 * berth[i].money / (berth[i].load_num + 0.01)) / (berth_ocean_dis_map[i][x][y] + 0.01);
        // fprintf(stderr, "%f\n", profit);
        // double profit = min(max_capacity - c_capacity, berth[i].load_num);
		if (e_profit < profit) {
			e_profit = profit;
			target = i;
		}
	}
	return target;
}


void Boat::moveBoat(char grid[N][N], int frame) {

    if(status == 1) return;
    
    switch(action) {
        case STAY:
        break;
        case SHIP:
            if (checkNextShipOk())
                printf("ship %d\n", id);
            else
                randMoveOrRotWithSafe();
        break;
        case ROT_0:
            if(checkNextRotOk(0))
                printf("rot %d %d\n", id, 0);
            else
                randMoveOrRotWithSafe();
        break;
        case ROT_1:
            if(checkNextRotOk(1))
                printf("rot %d %d\n", id, 1);
            else
                randMoveOrRotWithSafe();
        break;
        default:
            randMoveOrRotWithSafe();
        break;
    }
}

pair<Point, Point> Boat::getRect() {
    switch(dir) {
        case 0:
        return make_pair(Point(x, y), Point(x+1, y+2));
        break;
        case 1:
        return make_pair(Point(x-1, y-2), Point(x, y));
        break;
        case 2:
        return make_pair(Point(x-2, y), Point(x, y+1));
        break;
        case 3:
        return make_pair(Point(x, y-1), Point(x+2, y));
        break;
    }
    return make_pair(Point(x, y), Point(x, y));
}

pair<Point, Point> Boat::getRectUsePos(int p_x, int p_y, int d) {
    switch(d) {
        case 0:
        return make_pair(Point(p_x, p_y), Point(p_x+1, p_y+2));
        break;
        case 1:
        return make_pair(Point(p_x-1, p_y-2), Point(p_x, p_y));
        break;
        case 2:
        return make_pair(Point(p_x-2, p_y), Point(p_x, p_y+1));
        break;
        case 3:
        return make_pair(Point(p_x, p_y-1), Point(p_x+2, p_y));
        break;
    }
    return make_pair(Point(p_x, p_y), Point(p_x, p_y));
}

bool Boat::arrivedT() {
    return x == dstx && y == dsty;
}

bool Boat::arrivedBerth(char grid[N][N]) {
    return manhattanDistance(x, y, berth[go_berth_id].boat_x, berth[go_berth_id].boat_y) <= 6 && (grid[x][y] == 'K' || grid[x][y] == 'B');
}

pair<Point, int> Boat::getNextPointToRot(int rot)
{
    switch(dir) {
        case 0:
        if(rot) {
            return make_pair(Point(x+1, y+1), 2);
        } else {
            return make_pair(Point(x, y+2), 3);
        }
        break;
        case 1:
        if(rot) {
            return make_pair(Point(x-1, y-1), 3);
        } else {
            return make_pair(Point(x, y-2), 2);
        }
        break;
        case 2:
        if(rot) {
            return make_pair(Point(x, y-1), 1);
        } else {
            return make_pair(Point(x-2, y), 0);
        }
        break;
        case 3:
        if(rot) {
            return make_pair(Point(x+1, y-1), 0);
        } else {
            return make_pair(Point(x-2, y), 1);
        }
        break;
    }
    return make_pair(Point(x, y), dir);
}

static bool checkRect(const Point& left_top, const Point& right_bottom, int id) {

    bool nextNoCollision = true;
    boat[id].collide_with_id = -1;
    for(int x=left_top.x; x<=right_bottom.x; x++) {
        if(!nextNoCollision) break;
        for(int y=left_top.y; y<=right_bottom.y; y++) {
            if(x<0 || x>=N || y<0 || y>=N) return false;
            if(collision_property[grid[x][y]] != 0) {nextNoCollision = false; break;};
        }
    }
    if(nextNoCollision) return true;

    for(int i=0; i<boat_num; i++) {
        if(i == id) continue;

        if(manhattanDistance(boat[i].x, boat[i].y, boat[id].x, boat[id].y) >= 10) continue;
        auto [lt, rb] = boat[i].getRect();
        
        // 船体位于无碰撞区域
        nextNoCollision = true;
        for(int x=lt.x; x<=rb.x; x++) {
            if(!nextNoCollision) break;
            for(int y=lt.y; y<=rb.y; y++) {
                if(x<0 || x>=N || y<0 || y>=N) return false;
                if(collision_property[grid[x][y]] != 0) {nextNoCollision = false; break;};
            }
        }
        if(nextNoCollision) continue;;
        
        // 船体有相交
        if(!(lt.x > right_bottom.x || rb.x < left_top.x || lt.y > right_bottom.y || rb.y < left_top.y)) {
            boat[id].collide_with_id = i;
            return false;
        }
    }

    // 船体位于海洋区域
    for(int i=left_top.x; i<=right_bottom.x; i++) {
        for(int j=left_top.y; j<=right_bottom.y; j++) {
            if(i<0 || i>=N || j<0 || j>=N) return false;
            if(grid_property[grid[i][j]] != 2 && grid_property[grid[i][j]] != 3) {
                boat[id].collide_with_id = -1;
                return false;
            }
        }
    }
    return true;
}

bool Boat::checkNextShipOk()
{
    Point left_top, right_bottom;
    switch(dir) {
        case 0:
        left_top = Point(x, y+1);
        right_bottom = Point(x+1, y+3);
        break;
        case 1:
        left_top = Point(x-1, y-3);
        right_bottom = Point(x, y-1);
        break;
        case 2:
        left_top = Point(x-3, y);
        right_bottom = Point(x-1, y+1);
        break;
        case 3:
        left_top = Point(x+1, y-1);
        right_bottom = Point(x+3, y);
        break;
    }

    return checkRect(left_top, right_bottom, id);
}

bool Boat::checkNextRotOk(int r) {
    bool res = false;
    switch(dir) {
        case 0:
        if(r==0) {
            res = checkRect(Point(x, y+1), Point(x+2, y+2), id);
        } else {
            res = checkRect(Point(x-1, y+1), Point(x+1, y+2), id);
        }
        break;
        case 1:
        if(r==0) {
            res = checkRect(Point(x-2, y-2), Point(x, y-1), id);
        } else {
            res = checkRect(Point(x-1, y-2), Point(x+1, y-1), id);
        }
        break;
        case 2:
        if(r==0) {
            res = checkRect(Point(x-2, y), Point(x-1, y+2), id);
        } else {
            res = checkRect(Point(x-2, y-1), Point(x-1, y+1), id);
        }
        break;
        case 3:
        if(r==0) {
            res = checkRect(Point(x+1, y-2), Point(x+2, y), id);
        } else {
            res = checkRect(Point(x+1, y-1), Point(x+2, y+1), id);
        }
        break;
    }
    return res;
}


void Boat::followPath() {
    // 如果顺时针旋转后的点和路径点第三个点相同 并且第四个点和第三个点的方向和旋转后方向相同就顺时针旋转
    // 如果逆时针旋转后的点与第二个路径点和第三个路径点同向 就逆时针旋转

    // 逆时针旋转判断
    if(c_p + 1 <= path.size() - 1) {
        auto [next_p, next_dir] = getNextPointToRot(1);
        if(c_p + 1 == path.size() - 1 && next_p == path[c_p+1]) { // 到达目的地
            action = ROT_1;
            return;
        } else if(c_p + 2 <= path.size() - 1 && getDirection(path[c_p+2].x - path[c_p+1].x, path[c_p+2].y - path[c_p+1].y) == next_dir) { // 旋转后和后续路径方向一致
            action = ROT_1;
            path[c_p] = next_p;
            return;
        }
    }
    // 顺时针旋转判断
    if(c_p + 2 <= path.size() - 1) {
        auto [next_p, next_dir] = getNextPointToRot(0);
        if(c_p + 2 == path.size() - 1 && next_p == path[c_p+2]) { // 到达目的地
            action = ROT_0;
            return;
        } else if(c_p + 3 <= path.size() - 1 && getDirection(path[c_p+3].x - path[c_p+2].x, path[c_p+3].y - path[c_p+2].y) == next_dir) { // 旋转后和后续路径方向一致
            action = ROT_0;
            return;
        }
    }

    int d = getDirection(path[c_p + 1].x - path[c_p].x, path[c_p + 1].y - path[c_p].y);
    if(d == dir) action = SHIP;
    else {
        switch(dir) {
            case 0:
            if(d == 2) action = ROT_1;
            else if(d == 3) action = ROT_0;
            else action = RANDOM_ROT;
            break;
            case 1:
            if(d == 2) action = ROT_0;
            else if(d == 3) action = ROT_1;
            else action = RANDOM_ROT;
            break;
            case 2:
            if(d == 0) action = ROT_0;
            else if(d == 1) action = ROT_1;
            else action = RANDOM_ROT;
            break;
            case 3:
            if(d == 0) action = ROT_1;
            else if(d == 1) action = ROT_0;
            else action = RANDOM_ROT;
            break;
        }
    }
}

void Boat::randMoveOrRotWithSafe()
{
    // if(map_flag != 1 && collide_with_id != -1 && getPriority() > boat[collide_with_id].getPriority()) return; // stay
    if(rand() % 2 == 0) {
        if(checkNextShipOk()) printf("ship %d\n", id);
        else {
            if(checkNextRotOk(0)) printf("rot %d %d\n", id, 0);
            else if(checkNextRotOk(1)) printf("rot %d %d\n", id, 1);
        }
    } else {
        if(checkNextRotOk(0)) printf("rot %d %d\n", id, 0);
        else if(checkNextRotOk(1)) printf("rot %d %d\n", id, 1);
    }
}

pair<int, int> Boat::getMinDisT2P(int x, int y)
{
    int dis = N*N, select_id = -1;
    for(int i=0; i<delivery_point.size(); i++) {
        if(dis > T_ocean_dis_map[i][x][y]) {
            dis = T_ocean_dis_map[i][x][y];
            select_id = i;
        }
    }
    return make_pair(select_id, dis);
}
