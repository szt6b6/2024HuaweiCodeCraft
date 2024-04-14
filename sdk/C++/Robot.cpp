#include "Robot.h"
#include "utils.h"
#include "TaskAssigner.h"

extern Good* goods[N][N];

bool Robot::arrivePullDst(char grid[][N]) {
    if (manhattanDistance(x, y, msx, msy) <= 4 && grid[x][y] == 'B') return true;
    return false;
}

void Robot::update(char grid[][N], int gds[][N][2], int connectField[][N], 
                        int berth_dir_map[][N][N], int berth_dis_map[][N][N], 
                        vector<Robot>& robot, vector<Berth>& berth, vector<Boat>& boat, int frame)
{
    move = -1;
FREE_:
    if(taskT == FREE) { 
        priority = id;
        // int s = getMs();
        TaskAssigner::update(robot, berth, boat, grid, gds, connectField, berth_dir_map, berth_dis_map, frame);
        // int e = getMs();
        // fprintf(stderr, "frame %d, taskAssigner time: %d\n", frame_id, e - s);
    }
AVOID:
    if(is_avoid) {
        priority = id + 150;
        if(checkAroundHaveRobot(grid, robot, 1)) {
            moveWithNoRobotFront(grid, robot);
            return;
        } else {
            is_avoid = false;
        }
    }
GET_:
    if(taskT == GET) {
        priority = id + 50;
        if(arriveGetDst() && gds[mbx][mby][1] > 0) {
            printf("get %d\n", id);
            gds[mbx][mby][1] = 0;
            berth[dst_pull_berth_id].r_num++;
            taskT = PULL;
            money = gds[mbx][mby][0];
            path.clear();
            after_pull_at_berth_id = -1;
            if(type == 1 && good_nums == 0) {
                taskT = FREE;
                goto FREE_;
            }
            goto PULL_;
        } else if(arriveGetDst() && gds[mbx][mby][1] <= 0) {
            taskT = FREE;
            path.clear();
            after_pull_at_berth_id = -1;
            goto  FREE_;
            // TaskAssigner::update(robot, berth, boat, grid, gds, connectField, berth_dir_map, berth_dis_map, frame);
            // if(taskT == FREE) {
            //     // moveWithNoRobotFront(grid, robot);
            //     return;
            // }
        }

        move = goods[mbx][mby]->getDir(x, y);
        // int c_p = findPosition(path, x, y);

        // if (c_p > -1 && c_p < path.size() - 1)
        //     move = getDirection(path[c_p + 1].x - path[c_p].x, path[c_p + 1].y - path[c_p].y);
        // else {
        //     // mvoe = berth_dir_map[dst_pull_berth_id][x][y];
        //     path = bfs_path(grid, connectField, robot, x, y, mbx, mby, frame, id);
        //     // path = aStar(grid, connectField, robot, x, y, mbx, mby, frame, id);
        //     if(path.size() == 0) { // 被堵住了 没路可走 进去避让状态
        //         is_avoid = true;
        //         moveWithNoRobotFront(grid, robot);
        //         return;
        //     }
        //     c_p = findPosition(path, x, y);
        //     if (c_p != -1 && c_p < path.size() - 1)
        //         move = getDirection(path[c_p + 1].x - path[c_p].x, path[c_p + 1].y - path[c_p].y);
        //     else
        //         moveWithNoRobotFront(grid, robot);
        // }
    } 

PULL_:
    if(taskT == PULL) {
        priority = id + 100;
        if(arrivePullDst(grid)) {
            printf("pull %d\n", id);
            total_pull_money += money;
            total_pull_num += 1;
            berth[dst_pull_berth_id].r_num--;
            berth[dst_pull_berth_id].money += money;
            berth[dst_pull_berth_id].money_que.push(money);
            berth[dst_pull_berth_id].load_num++;
            berth[dst_pull_berth_id].btotal_load_num++;
            after_pull_at_berth_id = dst_pull_berth_id;
            taskT = FREE;
            money = 0;
            path.clear();
            goto FREE_;
            // TaskAssigner::update(robot, berth, boat, grid, gds, connectField, berth_dir_map, berth_dis_map, frame);
            // if(taskT == FREE) {
            //     // moveWithNoRobotFront(grid, robot);
            //     return;
            // }
            // else if(taskT == GET) {
            //     goto GET_;
            // }
        }
        // if(path.empty()) path = getPathFromBerthDirMap(berth_dir_map, mbx, mby, berth[dst_pull_berth_id].x, berth[dst_pull_berth_id].y, dst_pull_berth_id, false);

        if(berth[dst_pull_berth_id].forbid) {
            dst_pull_berth_id = TaskAssigner::findBerthToPULL(berth, boat, berth_dis_map, connectField, x, y, frame);
            msx = berth[dst_pull_berth_id].x;
            msy = berth[dst_pull_berth_id].y;
        }
        move = berth_dir_map[dst_pull_berth_id][x][y];

        // int c_p = findPosition(path, x, y);
        // if (c_p != -1 && c_p < path.size() - 1)
        //     move = getDirection(path[c_p + 1].x - path[c_p].x, path[c_p + 1].y - path[c_p].y);
        // else {
        //     //move = berth_dir_map[dst_pull_berth_id][x][y];
        //     path = bfs_path(grid, connectField, robot, x, y, msx, msy, frame, id);
        //     // path = aStar(grid, connectField, robot, x, y, msx, msy, frame, id);
        //     if(path.size() == 0) { // 被堵住了 没路可走 进去避让状态
        //         is_avoid = true;
        //         moveWithNoRobotFront(grid, robot);
        //         return;
        //     }
        //     c_p = findPosition(path, x, y);
        //     if (c_p > -1 && c_p < path.size() - 1)
        //         move = getDirection(path[c_p + 1].x - path[c_p].x, path[c_p + 1].y - path[c_p].y);
        //     else
        //         moveWithNoRobotFront(grid, robot);
        // }
    }
}

void Robot::setBSxy(int bx, int by, int sx, int sy) {
    mbx = bx;
    mby = by;
    msx = sx;
    msy = sy;
}

void Robot::moveWithNoRobotFront(char grid[][N], vector<Robot> &robot) {

    if(collision_property[grid[x][y]] == 0) {move = -1; return;}
    vector<bool> available_dir(4, false);
    // gather next point other robot at
    // scene one. distance 2, move to same point
    vector<Point> next_point;
    for(int i=0; i<robot_num; i++) {
        if(i == id) continue;
        if(robot[i].move != -1) {
            next_point.push_back(robot[i].getNextPoint());
        } else {
            next_point.push_back({robot[i].x, robot[i].y});
        }
    }

    for(int i=0; i<4; i++) {
        Point next = {x + directions[i].x, y + directions[i].y};
        if(next.x < 0 || next.x >= N || next.y < 0 || next.y >= N) continue;
        if(grid_property[grid[next.x][next.y]] != 1 && grid_property[grid[next.x][next.y]] != 3) continue;
        if(collision_property[grid[next.x][next.y]] == 0 && collision_property[grid[x][y]] == 0) {available_dir[i] == true; continue;}
        bool c = true;
        for(auto p : next_point) {
            if(next == p) { c = false; break;}
        }
        if(c) available_dir[i] = true;
    }

    // scene two, distance 1, move toward
    Point cur = {x , y};
    for(int i=0; i<robot_num; i++) {
        if(i == id) continue;
        if(robot[i].move != -1) {
            if(cur == robot[i].getNextPoint()) {
                int x_gap = robot[i].x - x;
                int y_gap = robot[i].y - y;
                available_dir[getDirection(x_gap, y_gap)] = false;
            }
        }
    }

    int i = rand() % 4, c = 0;
    while(!available_dir[i] && c < 4) {i = (i + 1) % 4; c++;}
    if(available_dir[i]) move = i;
    else move = -1;
}

bool Robot::checkAroundHaveRobot(char grid[][N], vector<Robot> &robot, int d)
{
    // around 8 grids
    for(int i=x-d; i<x+d; i++){
        for(int j=y-d; j<y+d; j++){
            if(i == x && j == y) continue;
            if(i<0 || i>=N || j<0 || j>=N) continue;
            for(int r=0; r<robot_num; r++) {
                if(r == id) continue;
                if(robot[r].priority > priority && robot[r].x == i && robot[r].y == j) {
                    return true;
                }
            }
        }
    }
    return false;
}

int Robot::gatherAroundRobotNum(vector<Robot> &robot, int d)
{
    int res = 0;
    for(int i=x-d; i<x+d; i++){
        for(int j=y-d; j<y+d; j++){
            if(i<0 || i>=N || j<0 || j>=N) continue;
            for(int r=0; r<robot_num; r++) {
                if(i == robot[r].x && j == robot[r].y) {
                    res++;
                }
            }
        }
    }
    return res;
}

Point Robot::getNextPoint()
{
    if(move != -1) return {x+directions[move].x, y+directions[move].y};
    else return {x, y};
}
