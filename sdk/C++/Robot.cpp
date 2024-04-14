#include "Robot.h"
#include "utils.h"
#include "TaskAssigner.h"

int Robot::i = 0;


bool Robot::arrivePullDst(char grid[][N]) {
    if (manhattanDistance(x, y, msx, msy) <= 2 && grid[x][y] == 'B') return true;
    return false;
}

void Robot::update(char grid[][N], int gds[][N][2], bool gds_locked[][N], int connectField[][N], 
                        int berth_dir_map[berth_num][N][N], int berth_dis_map[berth_num][N][N], 
                        vector<Robot>& robot, vector<Berth>& berth, vector<Boat>& boat, int frame)
{
    move = -1;
    if(status == 0) {
        priority = 10000.0;
        moveWithNoRobotFront(grid, robot);
        LOG_see_crash(frame, id, robot is crashed);
        return;
    }
    if(taskT == FREE) { 
        priority = id;
        moveWithNoRobotFront(grid, robot);
        LOG_INFO(frame, id, robot is FREE); 
        return;
    }
AVOID:
    if(is_avoid) {
        priority = id + 1000;
        LOG_INFO(frame, id, robot is avoid);
        if(checkAroundHaveRobot(grid, robot, 2)) {
            LOG_INFO(frame, id, robot is avoid and have other around);
            moveWithNoRobotFront(grid, robot);
            return;
        } else {
            is_avoid = false;
        }
    }
GET_:
    if(taskT == GET) {
        priority = id + 10;
        if(arriveGetDst() && gds[mbx][mby][1] > 0) {
            LOG_INFO(frame, id, robot get good);
            printf("get %d\n", id);
            gds[mbx][mby][1] = 0;
            taskT = PULL;
            money = gds[mbx][mby][0];
            path.clear();
            after_pull_at_berth_id = -1;
            goto PULL_;
        } else if(arriveGetDst() && gds[mbx][mby][1] <= 0) {
            LOG_INFO(frame, id, good dissapper);
            taskT = FREE;
            gds_locked[mbx][mby] = false;
            path.clear();
            after_pull_at_berth_id = -1;
            TaskAssigner::update(robot, berth, boat, grid, gds, gds_locked, connectField, berth_dir_map, berth_dis_map, frame);
            if(taskT == FREE) {
                // moveWithNoRobotFront(grid, robot);
                return;
            }
        }
        int c_p = findPosition(path, x, y);

        if (path.size() - c_p + frame > get_deadline) { // 时间不够去拿货物
            taskT = FREE;
            path.clear();
            after_pull_at_berth_id = -1;
            TaskAssigner::update(robot, berth, boat, grid, gds, gds_locked, connectField, berth_dir_map, berth_dis_map, frame);
            if(taskT == FREE) {
                // moveWithNoRobotFront(grid, robot);
                return;
            }
        }

        if (c_p > -1 && c_p < path.size() - 1)
            move = getDirection(path[c_p + 1].x - path[c_p].x, path[c_p + 1].y - path[c_p].y);
        else {
            //move = berth_dir_map[dst_pull_berth_id][x][y];
            // path = bfs_path(grid, connectField, robot, x, y, mbx, mby, frame, id);
            path = aStar(grid, connectField, robot, x, y, mbx, mby, frame, id);
            if(path.size() == 0) { // 被堵住了 没路可走 进去避让状态
                is_avoid = true;
                moveWithNoRobotFront(grid, robot);
                return;
            }
            c_p = findPosition(path, x, y);
            if (c_p != -1 && c_p < path.size() - 1)
                move = getDirection(path[c_p + 1].x - path[c_p].x, path[c_p + 1].y - path[c_p].y);
            else
                moveWithNoRobotFront(grid, robot);
        }
        LOG_INFO(frame, id, robot get move);
    } 

PULL_:
    if(taskT == PULL) {
        priority = id + 100;
        if(arrivePullDst(grid)) {
            LOG_INFO(frame, id, robot pull goods);
            printf("pull %d\n", id);
            total_pull_money += money;
            total_load_num += 1;
            berth[dst_pull_berth_id].money += money;
            berth[dst_pull_berth_id].load_num++;
            berth[dst_pull_berth_id].btotal_load_num++;
            after_pull_at_berth_id = dst_pull_berth_id;
            taskT = FREE;
            money = 0;
            path.clear();
            TaskAssigner::update(robot, berth, boat, grid, gds, gds_locked, connectField, berth_dir_map, berth_dis_map, frame);
            if(taskT == FREE) {
                // moveWithNoRobotFront(grid, robot);
                return;
            }
            else if(taskT == GET) {
                goto GET_;
            }
        }

        int c_p = findPosition(path, x, y);
        if (c_p != -1 && c_p < path.size() - 1)
            move = getDirection(path[c_p + 1].x - path[c_p].x, path[c_p + 1].y - path[c_p].y);
        else {
            //move = berth_dir_map[dst_pull_berth_id][x][y];
             //path = bfs_path(grid, connectField, robot, x, y, msx, msy, frame, id);
            path = aStar(grid, connectField, robot, x, y, msx, msy, frame, id);
            if(path.size() == 0) { // 被堵住了 没路可走 进去避让状态
                is_avoid = true;
                moveWithNoRobotFront(grid, robot);
                return;
            }
            c_p = findPosition(path, x, y);
            if (c_p > -1 && c_p < path.size() - 1)
                move = getDirection(path[c_p + 1].x - path[c_p].x, path[c_p + 1].y - path[c_p].y);
            else
                moveWithNoRobotFront(grid, robot);
        }

        LOG_INFO(frame, id, robot pull move);
    }
}

void Robot::setBSxy(int bx, int by, int sx, int sy) {
    mbx = bx;
    mby = by;
    msx = sx;
    msy = sy;
}

void Robot::moveWithNoRobotFront(char grid[][N], vector<Robot> &robot) {

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
        if(next.x <= 0 || next.x > n || next.y <= 0 || next.y > n) continue;
        if(grid[next.x][next.y] == '#' || grid[next.x][next.y] == '*') continue;
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
            if(i<=0 || i>=N || j<=0 || j>=N) continue;
            for(int r=0; r<robot_num; r++) {
                if(r == id) continue;
                if(robot[r].x == i && robot[r].y == j) {
                    return true;
                }
            }
        }
    }
    return false;
}

Point Robot::getNextPoint()
{
    if(move != -1) return {x+directions[move].x, y+directions[move].y};
    else return {x, y};
}
