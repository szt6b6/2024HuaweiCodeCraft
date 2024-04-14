#include "utils.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <random>
#include <string.h>
extern bool boat_dir_can_go_map[4][N][N];
extern char grid[N][N];
extern int grid_cost[4][N][N];
extern int P_to_closest_berht[N][N];
extern int connectField[N][N];
extern Good* goods[N][N];

int frame_ob_map[max_robot_num][N][N]; // store robot position at specific frame
void setFrameObMap(vector<Robot>& robot, int frame, int r_id)
{
    for (int i = 0; i < robot_num; i++) {
        for(int j=0; j<N; j++)
            memset(frame_ob_map[i][j], 0, N);
    }

    for (int i=0; i<robot_num; i++) {
        if (i == r_id) continue;
        frame_ob_map[i][robot[i].x][robot[i].y] = frame;

        // if(robot[i].taskT != PULL) continue; // 只考虑将去卖的机器人路径为障碍
        int j = 0;
        // first find robot position
        for (; j < robot[i].path.size(); ++j) {
            if (robot[i].path[j].x == robot[i].x && robot[i].path[j].y == robot[i].y) {
                break;
            }
        }
        int t = frame;
        for (; j < robot[i].path.size(); ++j) {
            frame_ob_map[i][robot[i].path[j].x][robot[i].path[j].y] = t++;
        }
    }
}


int boat_frame_ob_map[max_boat_num][N][N]; // store robot position at specific frame

static void setBoatFrameValue(int d, int i, int x, int y, int t) {
    if (collision_property[grid[x][y]] == 0) return;

    Point left_top, right_bottom;
    switch(d) {
        case 0:
        left_top = Point(x, y);
        right_bottom = Point(x+1, y+2);
        break;
        case 1:
        left_top = Point(x-1, y-2);
        right_bottom = Point(x, y);
        break;
        case 2:
        left_top = Point(x-2, y);
        right_bottom = Point(x, y+1);
        break;
        case 3:
        left_top = Point(x, y-1);
        right_bottom = Point(x+2, y);
        break;
    }

    for(int x=left_top.x; x<=right_bottom.x; ++x) {
        for(int y=left_top.y; y<=right_bottom.y; ++y)
            boat_frame_ob_map[i][x][y] = t;
    }
}

void setBoatFrameObMap(vector<Boat>& boat, int frame, int b_id)
{
    for (int i = 0; i < boat_num; i++) {
        for(int j=0; j<N; j++)
            memset(boat_frame_ob_map[i][j], 0, N);
    }

    for (int i=0; i<boat_num; i++) {
        if (i == b_id) continue;

        auto [lt, rb] = boat[i].getRect();
        for(int x=lt.x; x<=rb.x; ++x) {
            for(int y=lt.y; y<=rb.y; ++y) {
                if(collision_property[grid[x][y]] != 0)
                    boat_frame_ob_map[i][x][y] = frame;
            }
        }

        size_t j = 0;
        // first find boat position
        for (; j < boat[i].path.size(); ++j) {
            if (boat[i].path[j].x == boat[i].x && boat[i].path[j].y == boat[i].y) {
                break;
            }
        }
        int t = frame;
        int d = boat[i].dir;
        for (; j < boat[i].path.size(); ++j) {
            if(collision_property[grid[boat[i].path[j].x][boat[i].path[j].y]] != 0)
                setBoatFrameValue(d, i, boat[i].path[j].x, boat[i].path[j].y, t);
            if(j + 1 < boat[i].path.size()) {
                d = getDirection(boat[i].path[j+1].x - boat[i].path[j].x, boat[i].path[j+1].y - boat[i].path[j].y);
            } else d = boat[i].dir;

            t += grid_time_property[grid[boat[i].path[j].x][boat[i].path[j].y]];
        }
    }
}

// 定义访问数组, use new allocte memory at stack for avoiding zhan overflow
bool visited[N][N];
// 定义父节点数组
vector<vector<Point>> parent(N, vector<Point>(N));
// g cost
int g_map[N][N];

// A*算法函数, return a path from (sx, sy) to (dx, dy) in map
vector<Point> aStar(char grid[][N], int connectField[][N], vector<Robot>& robot, int sx, int sy, int dx, int dy, int frame, int r_id) {
    int s = getMs();
    Point start(sx, sy), goal(dx, dy);

    // 定义优先队列
    priority_queue<Node, vector<Node>, greater<Node>> openSet;

    // 初始化起始节点
    openSet.push(Node(start, 0, manhattanDistance(start, goal)));

    // result path
    vector<Point> path;
    if(connectField[sx][sy] != connectField[dx][dy]) {
        return path;
    } // judge if in same connectField

    // 定义访问数组, use new allocte memory at stack for avoiding zhan overflow
    for (int i = 0; i < N; i++) {
        fill(visited[i], visited[i] + N, false);
        fill(g_map[i], g_map[i] + N, 0);
    }
    // 标记当前节点为已访问
    visited[sx][sy] = true;

    setFrameObMap(robot, frame, r_id);

    while (!openSet.empty()) {
        // 从优先队列中取出f值最小的节点
        Node current = openSet.top();
        g_map[current.point.x][current.point.y] = current.g;
        openSet.pop();

        // 检查是否到达目标点
        if (current.point.x == goal.x && current.point.y == goal.y) {
            // 构造路径
            Point currentPoint = goal;
            while (!(currentPoint.x == start.x && currentPoint.y == start.y)) {
                path.push_back(currentPoint);
                currentPoint = parent[currentPoint.x][currentPoint.y];
            }
            path.push_back(start); // do not need start point
            reverse(path.begin(), path.end());
            // int e = getMs();
            // fprintf(stderr, "A* spend %d\n", e - s);
            return path;
        }

        // 探索当前节点的邻居
        for (const Point& direction : directions) {
            Point neighbor(current.point.x + direction.x, current.point.y + direction.y);

            // 检查邻居是否在网格范围内
            if (neighbor.x < 0 || neighbor.x >= N || neighbor.y < 0 || neighbor.y >= N) continue;
            
            // 检查邻居是否可通行
            if(grid_property[grid[neighbor.x][neighbor.y]] != 1 && grid_property[grid[neighbor.x][neighbor.y]] != 3) continue;
            bool can_go = true;
            for(int i=0; i<robot_num; i++) {
                if(i == r_id) continue;
                if ((frame_ob_map[i][neighbor.x][neighbor.y] == current.g + frame + 1 && collision_property[grid[neighbor.x][neighbor.y]] == 1) || // 占据同一个位置或者相向而撞
                        (!(collision_property[grid[neighbor.x][neighbor.y]] == 0 && collision_property[grid[current.point.x][current.point.y]] == 0) && frame_ob_map[i][neighbor.x][neighbor.y] == current.g + frame && frame_ob_map[i][current.point.x][current.point.y] == current.g + frame + 1)) {
                            can_go = false;
                            break;
                        }
            }
            if(!can_go) continue;
            // 检查邻居是否已访问
            if(visited[neighbor.x][neighbor.y]) {
                int neightbor_g = g_map[neighbor.x][neighbor.y];
                if(neightbor_g > current.g + 1) {
                    parent[neighbor.x][neighbor.y] = current.point;
                }
            } else {
                visited[neighbor.x][neighbor.y] = true;
                parent[neighbor.x][neighbor.y] = current.point;
                if(grid[neighbor.x][neighbor.y] == 'B') {
                    openSet.push(Node(neighbor, current.g, manhattanDistance(neighbor, goal)));
                } else
                    openSet.push(Node(neighbor, current.g+1, manhattanDistance(neighbor, goal)));
            }            
        }
    }
    // int e = getMs();
    // fprintf(stderr, "A* spend %d\n", e - s);
    // 如果无法到达目标点，返回空路径
    return path;
}

// use same data with bfs for saving memory
vector<Point> bfs_path(char grid[][N], int connectField[][N], vector<Robot> &robot, int sx, int sy, int dx, int dy, int frame, int r_id)
{
    int s = getMs();
    Point start(sx, sy), goal(dx, dy);

    // 定队列
    queue<Node> openSet;

    // 初始化起始节点
    openSet.push(Node(start, 0, manhattanDistance(start, goal)));

    // result path
    vector<Point> path;

    if(connectField[sx][sy] != connectField[dx][dy]) {
        return path;
    } // judge if in same connectField

    // 定义访问数组, use new allocte memory at stack for avoiding zhan overflow
    for (int i = 0; i < N; i++) {
        fill(visited[i], visited[i] + N, false);
    }
    // 标记当前节点为已访问
    visited[start.x][start.y] = true;

    setFrameObMap(robot, frame, r_id);

    while (!openSet.empty()) {
        // 从优先队列中取出f值最小的节点
        Node current = openSet.front();
        openSet.pop();

        // 检查是否到达目标点
        if (current.point.x == goal.x && current.point.y == goal.y 
                || (robot[r_id].taskT == PULL && manhattanDistance(current.point, goal) <= 4 && grid[current.point.x][current.point.y] == 'B')) {
            // 构造路径
            Point currentPoint = current.point;
            while (!(currentPoint.x == start.x && currentPoint.y == start.y)) {
                path.push_back(currentPoint);
                currentPoint = parent[currentPoint.x][currentPoint.y];
            }
            path.push_back(start); // do not need start point
            reverse(path.begin(), path.end());
            // int e = getMs();
            // fprintf(stderr, "bfs found, spend %d ms\n", e - s);
            return path;
        }

        // 探索当前节点的邻居
        for (const Point& direction : directions) {
            Point neighbor(current.point.x + direction.x, current.point.y + direction.y);

            // 检查邻居是否在网格范围内
            if (neighbor.x < 0 || neighbor.x >= N || neighbor.y < 0 || neighbor.y >= N) continue;
            
            // 检查邻居是否可通行
            if(grid_property[grid[neighbor.x][neighbor.y]] != 1 && grid_property[grid[neighbor.x][neighbor.y]] != 3) continue;
            bool can_go = true;
            for(int i=0; i<robot_num; i++) {
                if(i == r_id) continue;
                if ((frame_ob_map[i][neighbor.x][neighbor.y] == current.g + frame + 1 && collision_property[grid[neighbor.x][neighbor.y]] == 1) || // 占据同一个位置或者相向而撞
                        (!(collision_property[grid[neighbor.x][neighbor.y]] == 0 && collision_property[grid[current.point.x][current.point.y]] == 0) && frame_ob_map[i][neighbor.x][neighbor.y] == current.g + frame && frame_ob_map[i][current.point.x][current.point.y] == current.g + frame + 1)) {
                            can_go = false;
                            break;
                        }
            }
            if(!can_go) continue;
            // check neightbot position if exist robot with status == 0, consider that robot as obstacle
            //if(checkPosExistBrokenRobot(robot, neighbor.x, neighbor.y)) continue;

            // 检查邻居是否已访问
            if(visited[neighbor.x][neighbor.y]) continue;
  
            visited[neighbor.x][neighbor.y] = true;
            parent[neighbor.x][neighbor.y] = current.point;
            openSet.push(Node(neighbor, current.g+1, manhattanDistance(neighbor, goal)));
        
        }
    }
    // int e = getMs();
    // fprintf(stderr, "bfs found, spend %d ms\n", e - s);
    // 如果无法到达目标点，返回空路径
    return path;
}

vector<Point> boat_bfs_path(char grid[][N], int connectField[][N], vector<Boat>& boat, int sx, int sy, int dx, int dy, int frame, int b_id)
{
    int s = getMs();
    const Point start(sx, sy), goal(dx, dy);

    // 定队列
    // queue<BFS_Node> openSet;
    priority_queue<BFS_Node, vector<BFS_Node>, greater<BFS_Node>> openSet;

    // 初始化起始节点
    openSet.push(BFS_Node(start, 0, manhattanDistance(start, goal)));

    // result path
    vector<Point> path;

    // if(connectField[sx][sy] != connectField[dx][dy]) {
    //     return path;
    // } // judge if in same connectField

    setBoatFrameObMap(boat, frame, b_id);

    // 定义访问数组, use new allocte memory at stack for avoiding zhan overflow
    for (int i = 0; i < N; i++) {
        fill(visited[i], visited[i] + N, false);
        fill(g_map[i], g_map[i] + N, 0);
    }

    while (!openSet.empty()) {
        BFS_Node current = openSet.top();
        g_map[current.point.x][current.point.y] = current.g;
        openSet.pop();

        // 检查是否到达目标点
        if (current.point.x == goal.x && current.point.y == goal.y) {
            // 构造路径
            Point currentPoint = goal;
            while (!(currentPoint.x == start.x && currentPoint.y == start.y)) {
                path.push_back(currentPoint);
                currentPoint = parent[currentPoint.x][currentPoint.y];
            }
            path.push_back(start); // do not need start point
            reverse(path.begin(), path.end());
            // int e = getMs();
            // fprintf(stderr, "bfs found, spend %d ms\n", e - s);
            return path;
        }

        // 标记当前节点为已访问
        visited[current.point.x][current.point.y] = true;

        // 探索当前节点的邻居
        for (int c_dir = 0; c_dir < 4; ++c_dir) {

            const Point& direction = directions[c_dir];
            Point neighbor(current.point.x + direction.x, current.point.y + direction.y);

            // 检查邻居是否在网格范围内
            if (neighbor.x < 0 || neighbor.x >= N || neighbor.y < 0 || neighbor.y >= N) continue;
            if (!boat_dir_can_go_map[c_dir][neighbor.x][neighbor.y]) continue;

            if(visited[neighbor.x][neighbor.y]) {
                int neightbor_g = g_map[neighbor.x][neighbor.y];
                if(neightbor_g > current.g + grid_cost[c_dir][neighbor.x][neighbor.y]) {
                    parent[neighbor.x][neighbor.y] = current.point;
                }
                continue;
            }

            // 检查邻居是否可通行
            if(grid_property[grid[neighbor.x][neighbor.y]] != 2 && grid_property[grid[neighbor.x][neighbor.y]] != 3) continue;
            
            bool can_go = true;
            auto [lt, rb] = Boat::getRectUsePos(neighbor.x, neighbor.y, c_dir);
            for(int i=0; i<boat_num; i++) {
                if(i == b_id) continue;
                for(int ii=lt.x; ii<=rb.x; ii++) {
                    for(int jj=lt.y; jj<=rb.y; jj++) {
                        if(ii < 0 || ii >= N || jj < 0 || jj >= N) continue;
                        if(abs(boat_frame_ob_map[i][ii][jj] - current.g - frame) <= 2) {
                            can_go = false;
                            break;
                        }
                    }
                }
            }
            if(!can_go) continue;

            visited[neighbor.x][neighbor.y] = true;
            parent[neighbor.x][neighbor.y] = current.point;
            openSet.push(BFS_Node(neighbor, current.g+grid_cost[c_dir][neighbor.x][neighbor.y], manhattanDistance(neighbor, goal)));
        }
    }
    // int e = getMs();
    // fprintf(stderr, "bfs found, spend %d ms\n", e - s);
    // 如果无法到达目标点，返回空路径
    return path;
}

vector<Point> getPathFromBerthDirMap(int berth_dir_map[max_berth_num][N][N], int mbx, int mby, int x, int y, int berth_id, bool rev=true)
{
    vector<Point> path;
    int c_x = mbx, c_y = mby;
    while (c_x != x || c_y != y) {
        path.push_back({ c_x, c_y });
        Point dir = directions[berth_dir_map[berth_id][c_x][c_y]];
        c_x += dir.x;
        c_y += dir.y;
    }
    path.push_back({ x, y });
    // if(x == 61 && y == 113) {
    //     fprintf(stderr, "(51, 112) dir: (%d, %d)\n", directions[berth_dir_map[berth_id][51][112]].x, directions[berth_dir_map[berth_id][51][112]].y);
    //     fprintf(stderr, "see path in dir map: ");
    //     for(int i = 0; i < path.size(); i++) {
    //         fprintf(stderr, "(%d,%d)->", path[i].x, path[i].y);
    //     }
    //     fprintf(stderr, "\n");
    // }
    if(rev) reverse(path.begin(), path.end());
    return path;
}

Point randomPoint(int n, char grid[][N])
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, n);
    int x = dis(gen);
    int y = dis(gen);
    while(grid[x][y] != '.')
    {
        x = dis(gen);
        y = dis(gen);
    }
    return Point(x, y);
}

int getDirection(int x, int y)
{
    if(x == 0 && y == 1) return 0; // right
    if(x == 0 && y == -1) return 1; // left
    if(x == -1 && y == 0) return 2; // up
    if(x == 1 && y == 0) return 3; // down
    return -1;
}

size_t findPosition(vector<Point>& path, int x, int y){
    for(size_t i = 0; i < path.size(); i++){
        if(path[i].x == x && path[i].y == y){
            return i;
        }
    }
    return -1;
}

#include <stdio.h>
// 从文件中读取地图数据到二维数组
void readMapFromFile(const char *filename, char map[N][N]) {
    FILE *file = fopen(filename, "r");

    if (file == NULL) {
        perror("Error opening file");
        exit(EXIT_FAILURE);
    }

    // 读取地图数据
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            fscanf(file, " %c", &map[i][j]);  // 注意空格，避免读取换行符等
        }
    }

    fclose(file);
}


void bfs(char grid[][N], int connectField[][N], vector<vector<bool>>& searched, int i, int j, int field_n) {
    queue<Point> que;
    que.push({i, j});
    while(!que.empty()) {
        Point c_p = que.front();
        que.pop();
        connectField[c_p.x][c_p.y] = field_n;

        for(auto dir : directions) {
            int n_x = c_p.x + dir.x, n_y = c_p.y + dir.y;
            if(n_x < 0 || n_x >= N || n_y < 0 || n_y >= N) continue;
            if(searched[n_x][n_y]) continue;
            if(grid_property[grid[n_x][n_y]] != 1 && grid_property[grid[n_x][n_y]] != 3) continue;
            searched[n_x][n_y] = true;
            que.push({n_x, n_y});
        }
    }
}


void connectFieldSplit(char grid[][N], int connectField[][N])
{
    static int filed_n = 1;
    // bfs filled
    vector<vector<bool>> searched(N, vector<bool>(N, false));
    
    for(int i=0; i< N; i++) {
        for(int j=0; j< N; j++) {
            if(searched[i][j]) continue;
            if(grid_property[grid[i][j]] != 1 && grid_property[grid[i][j]] != 3) continue;
            bfs(grid, connectField, searched, i, j, filed_n);
            filed_n++;
        }
    }
}

void ocean_bfs(char grid[][N], int connectField[][N], vector<vector<bool>>& searched, int i, int j, int field_n) {
    queue<Point> que;
    que.push({i, j});
    while(!que.empty()) {
        Point c_p = que.front();
        que.pop();
        connectField[c_p.x][c_p.y] = field_n;

        for(auto dir : directions) {
            int n_x = c_p.x + dir.x, n_y = c_p.y + dir.y;
            if(n_x < 0 || n_x >= N || n_y < 0 || n_y >= N) continue;
            if(searched[n_x][n_y]) continue;
            if(grid_property[grid[n_x][n_y]] != 2 && grid_property[grid[n_x][n_y]] != 3) continue;
            searched[n_x][n_y] = true;
            que.push({n_x, n_y});
        }
    }
}


void ocean_connectFieldSplit(char grid[][N], int ocean_connectField[][N])
{
    static int ocean_filed_n = 1;
    // bfs filled
    vector<vector<bool>> searched(N, vector<bool>(N, false));
    
    for(int i=0; i< N; i++) {
        for(int j=0; j< N; j++) {
            if(searched[i][j]) continue;
            if(grid_property[grid[i][j]] != 2 && grid_property[grid[i][j]] != 3) continue;
            ocean_bfs(grid, ocean_connectField, searched, i, j, ocean_filed_n);
            ocean_filed_n++;
        }
    }
}

// get millisecond
long long getMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void delay(int s) {
    using namespace std::this_thread; // sleep_for, sleep_until
    using namespace std::chrono; // nanoseconds, system_clock, seconds
    sleep_for(seconds(s));
}

// euler distance
double eulerDistance(int x, int y, int x1, int y1) {
    return sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1));
}

// 计算两点之间的曼哈顿距离
int manhattanDistance(const Point& a, const Point& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

int manhattanDistance(int x, int y, int x1, int y1) {
    return abs(x - x1) + abs(y - y1);
}


void initBerthDirAndDisMap_bfs(int berth_dir_map[][N][N], int berth_dis_map[][N][N], char grid[][N], int i, int j, int b_id) {
    queue<Point> que;
    queue<int> dis_que;
    que.push({ i, j });
    dis_que.push(1);

    vector<vector<bool>> searched(N, vector<bool>(N, false));
    searched[i][j] = true;
    while (!que.empty()) {
        Point c_p = que.front();
        que.pop();
        int dis = dis_que.front();
        dis_que.pop();
        berth_dis_map[b_id][c_p.x][c_p.y] = dis;
        // fprintf(stderr, "dis %d\n", dis);

        for (int d_i = 0; d_i < 4; d_i++) {
            int n_x = c_p.x + directions[d_i].x, n_y = c_p.y + directions[d_i].y;
            if (n_x < 0 || n_x >= N || n_y < 0 || n_y >= N) continue;
            if (grid_property[grid[n_x][n_y]] != 1 && grid_property[grid[n_x][n_y]] != 3) continue;
            if (searched[n_x][n_y]) continue;
            searched[n_x][n_y] = true;

            // if dir is up, then this point should go down to berth, if left then should go right
            if (d_i == 0) // right
                berth_dir_map[b_id][n_x][n_y] = 1;
            else if (d_i == 1) // left
                berth_dir_map[b_id][n_x][n_y] = 0;
            else if (d_i == 3) // down
                berth_dir_map[b_id][n_x][n_y] = 2;
            else // up
                berth_dir_map[b_id][n_x][n_y] = 3;
            que.push({ n_x, n_y });
            dis_que.push(dis + grid_time_property[grid[n_x][n_y]]);
        }
    }
}

/*
    for each berth, first need to know robot to which point to pull goods, 
    then from this point using bfs search gather direction and set berth_dir_map
*/
void initBerthDirAndDisMap(int berth_dir_map[][N][N], int berth_dis_map[][N][N], char grid[][N], vector<Berth>& berth)
{
    for (int i = 0; i < berth_num; i++) {
        initBerthDirAndDisMap_bfs(berth_dir_map, berth_dis_map, grid, berth[i].x, berth[i].y, i);
    }

    for(int i=0; i<N; i++) {
        for(int j=0; j<N; j++) {
            int select_b = -1, select_dis = INT_MAX;
            for(int b=0; b<berth_num; b++) {
                if(berth_dis_map[b][i][j] != 0 && berth_dis_map[b][i][j] < select_dis) {
                    select_b = b;
                    select_dis = berth_dis_map[b][i][j];
                } 
            }
            if(select_b != -1) {
                P_to_closest_berht[i][j] = select_b;
                ++berth[select_b].total_p_bind_berth;
                berth[select_b].avg_dis_to_pull += select_dis;
            }
        }
    }

    for(int i=0; i<berth_num; i++) berth[i].avg_dis_to_pull /= (berth[i].total_p_bind_berth + 0.01);
}

void initOceanDisMap_bfs(int dis_map[][N][N], char grid[][N], int i, int j, int b_id) {
    queue<Point> que;
    queue<int> dis_que;
    que.push({ i, j });
    dis_que.push(1);

    vector<vector<bool>> searched(N, vector<bool>(N, false));
    searched[i][j] = true;
    while (!que.empty()) {
        Point c_p = que.front();
        que.pop();
        int dis = dis_que.front();
        dis_que.pop();
        dis_map[b_id][c_p.x][c_p.y] = dis;
        // fprintf(stderr, "dis %d\n", dis);

        for (int d_i = 0; d_i < 4; d_i++) {
            int n_x = c_p.x + directions[d_i].x, n_y = c_p.y + directions[d_i].y;
            if (n_x < 0 || n_x >= N || n_y < 0 || n_y >= N) continue;
            if (grid_property[grid[n_x][n_y]] != 2 && grid_property[grid[n_x][n_y]] != 3) continue;
            if (searched[n_x][n_y]) continue;
            searched[n_x][n_y] = true;

            que.push({ n_x, n_y });
            dis_que.push(dis + grid_time_property[grid[n_x][n_y]]);

        }
    }
}

void initOceanBerthDisMap(int ocean_berth_dis_map[][N][N], char grid[][N], vector<Berth>& berth)
{
    for (int i = 0; i < berth_num; i++) {
        initOceanDisMap_bfs(ocean_berth_dis_map, grid, berth[i].x, berth[i].y, i);
    }
}

void initGridCost(char grid[][N], int grid_cost[][N][N])
{
    for(int i=0; i<N; i++) {
        for(int j=0; j<N; j++) {
            if(grid_cost[0][i][j] == 0) grid_cost[0][i][j] = grid_time_property[grid[i][j]];
            if(grid_cost[1][i][j] == 0) grid_cost[1][i][j] = grid_time_property[grid[i][j]];
            if(grid_cost[2][i][j] == 0) grid_cost[2][i][j] = grid_time_property[grid[i][j]];
            if(grid_cost[3][i][j] == 0) grid_cost[3][i][j] = grid_time_property[grid[i][j]];

            
            if(grid[i][j] == '~' || grid[i][j] == 'c' || grid[i][j] =='C' || grid[i][j] =='S' || grid[i][j] == 'K') {
                // 0
                for(int x=max(0, i-1); x<=i; x++)
                    for(int y=max(0, j-2); y<=j; y++) 
                        grid_cost[0][x][y] = max(grid_cost[0][x][y], grid_time_property[grid[i][j]]);
                        
                // 1
                for(int x=i; x<=min(N-1, i+1); x++)
                    for(int y=j; y<=min(N-1, j+2); y++)
                        grid_cost[1][x][y] = max(grid_cost[1][x][y], grid_time_property[grid[i][j]]);

                // 2
                for(int x=i; x<=min(N-1, i+2); x++)
                    for(int y=max(0, j-1); y<=j; y++)
                        grid_cost[2][x][y] = max(grid_cost[2][x][y], grid_time_property[grid[i][j]]);

                // 3
                for(int x=max(0, i-2); x<=i; x++)
                    for(int y=j; y<=min(N-1, j+1); y++)
                        grid_cost[3][x][y] = max(grid_cost[3][x][y], grid_time_property[grid[i][j]]);
            }
        }
    }
}

void initOceanTDisMap(int ocean_T_dis_map[][N][N], char grid[][N], vector<Point>& T)
{
    for (int i = 0; i < T.size(); i++) {
        initOceanDisMap_bfs(ocean_T_dis_map, grid, T[i].x, T[i].y, i);
    }
}

int countNearRobotNum(int x, int y, char grid[N][N], vector<Robot>& robot)
{
    int count = 0;
    for (int i = x - 2; i <= x + 2; ++i) {
        for (int j = y - 2; j <= y + 2; ++j) {
            if (i < 0 || i >= N || j < 0 || j >= N) continue;
            for (int k = 0; k < robot_num; ++k) {
                if (robot[k].x == x && robot[k].y == y) ++count;
            }
        }
    }
    return count;
}


bool checkCollisionBetwRobot(vector<Robot>& robot, int r1_id, int r2_id)
{
    int dis = manhattanDistance({ robot[r1_id].x, robot[r1_id].y }, { robot[r2_id].x, robot[r2_id].y });
    if (dis > 2) return false;

    int dir1 = robot[r1_id].move, dir2 = robot[r2_id].move;
    Point r1(robot[r1_id].x, robot[r1_id].y), r2(robot[r2_id].x, robot[r2_id].y);
    Point r1_next, r2_next;
    if (dir1 != -1) {
        r1_next = { robot[r1_id].x + directions[dir1].x, robot[r1_id].y + directions[dir1].y };
    }
    else {
        r1_next = { robot[r1_id].x, robot[r1_id].y };
    }

    if (dir2 != -1) {
        r2_next = { robot[r2_id].x + directions[dir2].x, robot[r2_id].y + directions[dir2].y };
    }
    else {
        r2_next = { robot[r2_id].x, robot[r2_id].y };
    }

    if(collision_property[grid[r1_next.x][r1_next.y]] == 0 && collision_property[grid[r2_next.x][r2_next.y]] == 0) return false;

    if (r1_next == r2_next || (r1_next == r2 && r2_next == r1)) {
        // fprintf(stderr, "will collision %d, %d\n", r1_id, r2_id);
        return true;
    }

    return false;
}

void init_taskBindQue(priority_queue<RobotTaskProfit, vector<RobotTaskProfit>, greater<RobotTaskProfit>>& taskBindQue, int taskIds_map[N][N], char grid[N][N], vector<Robot>& robot, vector<Task>& taskQueue, int frame) {
    
    for(int i=0; i<N; i++) {
        for(int j=0; j<N; j++) {
            if(taskIds_map[i][j] == -1) continue;
            int taskIdInTaskQue = taskIds_map[i][j];
            Good* good = goods[i][j];
            for(int r=0; r<robot_num; r++) {
                if(robot[r].taskT == PULL) continue;
                if(connectField[i][j] != connectField[robot[r].x][robot[r].y]) continue;
                int buy_dis = good->getDis(robot[r].x, robot[r].y);
                int gap_frame = taskQueue[taskIdInTaskQue].deadline - frame - buy_dis;
                if (gap_frame > 0) {
                    // if(map_flag == 1) taskBindQue.push(RobotTaskProfit(r, taskIdInTaskQue, 1250.0 / (gap_frame + 4.5 * Berth::getMeanDisToGds()) * taskQueue[taskIdInTaskQue].money * 1.0 / (buy_dis + 0.25 * taskQueue[taskIdInTaskQue].to_dest_berth_dis)));
                    // else taskBindQue.push(RobotTaskProfit(r, taskIdInTaskQue, 1250.0 / (gap_frame +350) * taskQueue[taskIdInTaskQue].money * 1.0 / (buy_dis + 0.3*taskQueue[taskIdInTaskQue].to_dest_berth_dis)));
                    // taskBindQue.push(RobotTaskProfit(r, taskIdInTaskQue, -log(gap_frame + buy_dis + Berth::getMeanDisToGds()*1.1) + 1.6*log(taskQueue[taskIdInTaskQue].money) - log(buy_dis + 0.25 * taskQueue[taskIdInTaskQue].to_dest_berth_dis)));
                    if(map_flag == 1) {
                        taskBindQue.push(RobotTaskProfit(r, taskIdInTaskQue, 1250.0 / (gap_frame + buy_dis + Berth::getMeanDisToGds()) * taskQueue[taskIdInTaskQue].money * 1.0 / (buy_dis + 0.2 * taskQueue[taskIdInTaskQue].to_dest_berth_dis)));
                    } else if(map_flag == 2) {
                        taskBindQue.push(RobotTaskProfit(r, taskIdInTaskQue, 1250.0 / (gap_frame + + buy_dis + Berth::getMeanDisToGds()) * taskQueue[taskIdInTaskQue].money * 1.0 / (buy_dis + 0.25 * taskQueue[taskIdInTaskQue].to_dest_berth_dis)));
                    } else {
                        taskBindQue.push(RobotTaskProfit(r, taskIdInTaskQue, 1250.0 / (gap_frame + buy_dis + + Berth::getMeanDisToGds()) * taskQueue[taskIdInTaskQue].money * 1.0 / (buy_dis + 0.2 * taskQueue[taskIdInTaskQue].to_dest_berth_dis)));
                    }
                }
            }
        }
    }
}

void intiBoatDirCanGoMap(char grid[N][N], bool boat_dir_can_go_map[4][N][N])
{
    for(int i = 0; i < N; i++) {
        for(int j = 0; j < N; j++) {
            if(grid_property[grid[i][j]] == 2 || grid_property[grid[i][j]] == 3) {
                for(int d=0; d<4; d++)
                    boat_dir_can_go_map[d][i][j] = true;;
            }
        }
    }

    // 0
    for(int i = 0; i<N; i++) {
        for(int j=0; j<N; j++) {
            if(i == N-1) {
                boat_dir_can_go_map[0][i][j] = false;
            }
            if(j == N-1) {
                boat_dir_can_go_map[0][i][j] = false;
                boat_dir_can_go_map[0][i][j-1] = false;
            }
            if(grid_property[grid[i][j]] != 2 && grid_property[grid[i][j]] != 3) {
                for(int x=max(0, i-1); x<=i; x++)
                    for(int y=max(0, j-2); y<=j; y++)
                        if(grid[x][y] != 'B') boat_dir_can_go_map[0][x][y] = false;
            }
        }
    }
    // 1
    for(int i = N-1; i>=0; i--) {
        for(int j=N-1; j>=0; j--) {
            if(i == 0) {
                boat_dir_can_go_map[1][i][j] = false;
            }
            if(j == 0) {
                boat_dir_can_go_map[1][i][j] = false;
                boat_dir_can_go_map[1][i][j+1] = false;
            }
            if(grid_property[grid[i][j]] != 2 && grid_property[grid[i][j]] != 3) {
                for(int x=i; x<=min(N-1, i+1); x++)
                    for(int y=j; y<=min(N-1, j+2); y++)
                        if(grid[x][y] != 'B') boat_dir_can_go_map[1][x][y] = false;
            }
        }
    }
    // 2
    for(int i = N-1; i>=0; i--) {
        for(int j=0; j<N; j++) {
            if(i == 0) {
                boat_dir_can_go_map[2][i][j] = false;
                boat_dir_can_go_map[2][i+1][j] = false;
            }
            if(j == N-1) {
                boat_dir_can_go_map[2][i][j] = false;
            }
            if(grid_property[grid[i][j]] != 2 && grid_property[grid[i][j]] != 3) {
                for(int x=i; x<=min(N-1, i+2); x++)
                    for(int y=max(0, j-1); y<=j; y++)
                        if(grid[x][y] != 'B') boat_dir_can_go_map[2][x][y] = false;
            }
        }
    }
    // 3
    for(int i=0; i<N; i++) {
        for(int j=N-1; j>=0; j--) {
            if(i == N-1) {
                boat_dir_can_go_map[3][i][j] = false;
                boat_dir_can_go_map[3][i-1][j] = false;
            }
            if(j == 0) {
                boat_dir_can_go_map[3][i][j] = false;
            }
            if(grid_property[grid[i][j]] != 2 && grid_property[grid[i][j]] != 3) {
                for(int x=max(0, i-2); x<=i; x++)
                    for(int y=j; y<=min(N-1, j+1); y++)
                        if(grid[x][y] != 'B') boat_dir_can_go_map[3][x][y] = false;
            }
        }
    }
}

double getAvgNewGoodsSpeed()
{
    return total_added_goods_num / (frame_id + 0.01);
}

double getAvgRobotsPullSpeed()
{
    return total_pull_num / (frame_id + 0.01);
}

double getAvgBoatToSellSpeed()
{
    return Boat::total_sold_goods_num / (frame_id + 0.01);
}


void Good::init()
{
    for(int i=0; i<N; i++) fill(visited[i], visited[i]+N, false);
    queue<Node> que;
    que.push(Node({x, y}, 0, 0));
    visited[x][y] = true;
    while(!que.empty()) {
        Node c_p = que.front();
        que.pop();
        *(dis_map + c_p.point.x * N + c_p.point.y) = c_p.g;

        for(int d_i=0; d_i<4; d_i++) {
            const Point& dir = directions[d_i]; 
            int n_x = c_p.point.x + dir.x, n_y = c_p.point.y + dir.y;
            if(n_x < 0 || n_x >= N || n_y < 0 || n_y >= N) continue;
            if(visited[n_x][n_y]) continue;
            if(grid_property[grid[n_x][n_y]] != 1 && grid_property[grid[n_x][n_y]] != 3) continue;
            visited[n_x][n_y] = true;
            // if dir is up, then this point should go down to berth, if left then should go right
            if (d_i == 0) // right
                *(dir_map + n_x * N + n_y) = 1;
            else if (d_i == 1) // left
                *(dir_map + n_x * N + n_y) = 0;
            else if (d_i == 3) // down
                *(dir_map + n_x * N + n_y) = 2;
            else // up
                *(dir_map + n_x * N + n_y) = 3;

            que.push(Node({n_x, n_y}, c_p.g+1, 0));
        }
    }
}
