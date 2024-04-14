#include "utils.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <random>
#include <string.h>

extern bool map1;

bool checkPosExistBrokenRobot(vector<Robot>& robot, int x, int y) {
    for (auto& r : robot) {
        if (r.status == 0 && r.x == x && r.y == y) {
            return true;
        }
    }
    return false;
}

int frame_ob_map[robot_num][N][N]; // store robot position at specific frame
void setFrameObMap(vector<Robot>& robot, int frame, int r_id)
{
    static int priority = INT_MAX;
    priority--;

    for (int i = 0; i < robot_num; i++) {
        for(int j=0; j<N; j++)
            memset(frame_ob_map[i][j], 0, N);
    }

    for (int i=0; i<robot_num; i++) {
        if (i == r_id) continue;
        if (robot[i].status == 0) {
            frame_ob_map[i][robot[i].x][robot[i].y] = frame;
            continue;
        }
        if(robot[i].taskT != PULL) continue; // 只考虑将去卖的机器人路径为障碍
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
    robot[r_id].priority = priority;
}

// 定义访问数组, use new allocte memory at stack for avoiding zhan overflow
bool visited[N][N];
// 定义父节点数组
vector<vector<Point>> parent(N, vector<Point>(N));
// g cost
int g_map[N][N];

// A*算法函数, return a path from (sx, sy) to (dx, dy) in map
vector<Point> aStar(char grid[][N], int connectField[][N], vector<Robot>& robot, int sx, int sy, int dx, int dy, int frame, int r_id) {
    //int s = getMs();
    Point start(sx, sy), goal(dx, dy);

    // 定义优先队列
    priority_queue<Node, vector<Node>, greater<Node>> openSet;

    // 初始化起始节点
    openSet.push(Node(start, 0, manhattanDistance(start, goal)));

    // result path
    vector<Point> path;
    int allow_timeframe_ob = 2;
    if(connectField[sx][sy] != connectField[dx][dy]) {
        return path;
    } // judge if in same connectField

    // 定义访问数组, use new allocte memory at stack for avoiding zhan overflow
    for (int i = 0; i < N; i++) {
        fill(visited[i], visited[i] + N, false);
        fill(g_map[i], g_map[i] + N, 0);
    }

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
            //int e = getMs();
            //fprintf(stderr, "A* spend %d\n", e - s);
            return path;
        }

        // 标记当前节点为已访问
        visited[current.point.x][current.point.y] = true;

        // 探索当前节点的邻居
        for (const Point& direction : directions) {
            Point neighbor(current.point.x + direction.x, current.point.y + direction.y);

            // 检查邻居是否在网格范围内
            if (neighbor.x <= 0 || neighbor.x > n || neighbor.y <= 0 || neighbor.y > n) continue;
            
            // 检查邻居是否可通行
            if(grid[neighbor.x][neighbor.y] == '#' || grid[neighbor.x][neighbor.y] == '*') continue;
            // bool can_go = true;
            // for(int i=0; i<robot_num; i++) {
            //     if(i == r_id) continue;
            //     if (frame_ob_map[i][neighbor.x][neighbor.y] == current.g + frame + 1 || // 占据同一个位置或者相向而撞
            //             (frame_ob_map[i][neighbor.x][neighbor.y] == current.g + frame && frame_ob_map[i][current.point.x][current.point.y] == current.g + frame + 1)) {
            //                 allow_timeframe_ob--;
            //                 can_go = false;
            //                 break;
            //             }
            // }
            // if(map1 && !allow_timeframe_ob) continue;
            // if (!map1 && !can_go) continue;
            // check neightbot position if exist robot with status == 0, consider that robot as obstacle
            //if(checkPosExistBrokenRobot(robot, neighbor.x, neighbor.y)) continue;

            // 检查邻居是否已访问
            if(visited[neighbor.x][neighbor.y]) {
                int neightbor_g = g_map[neighbor.x][neighbor.y];
                if(neightbor_g > current.g + 1) {
                    parent[neighbor.x][neighbor.y] = current.point;
                }
            } else {
                visited[neighbor.x][neighbor.y] = true;
                parent[neighbor.x][neighbor.y] = current.point;
                openSet.push(Node(neighbor, current.g+1, manhattanDistance(neighbor, goal)));
            }            
        }
    }
    //int e = getMs();
    //fprintf(stderr, "A* spend %d\n", e - s);
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
        for (int j = 0; j < N; j++) {
            visited[i][j] = false;
        }
    }

    setFrameObMap(robot, frame, r_id);

    while (!openSet.empty()) {
        // 从优先队列中取出f值最小的节点
        Node current = openSet.front();
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
        for (const Point& direction : directions) {
            Point neighbor(current.point.x + direction.x, current.point.y + direction.y);

            // 检查邻居是否在网格范围内
            if (neighbor.x <= 0 || neighbor.x > n || neighbor.y <= 0 || neighbor.y > n) continue;
            
            // 检查邻居是否可通行
            if(grid[neighbor.x][neighbor.y] == '#' || grid[neighbor.x][neighbor.y] == '*') continue;
            bool can_go = true;
            for(int i=0; i<robot_num; i++) {
                if(i == r_id) continue;
                if (frame_ob_map[i][neighbor.x][neighbor.y] == current.g + frame + 1 || // 占据同一个位置或者相向而撞
                        (frame_ob_map[i][neighbor.x][neighbor.y] == current.g + frame && frame_ob_map[i][current.point.x][current.point.y] == current.g + frame + 1)) {
                            can_go = false;
                            break;
                        }
            }
            if(!can_go) continue;
            // check neightbot position if exist robot with status == 0, consider that robot as obstacle
            //if(checkPosExistBrokenRobot(robot, neighbor.x, neighbor.y)) continue;

            // 检查邻居是否已访问
            if(!visited[neighbor.x][neighbor.y]) {
                visited[neighbor.x][neighbor.y] = true;
                parent[neighbor.x][neighbor.y] = current.point;
                openSet.push(Node(neighbor, current.g+1, manhattanDistance(neighbor, goal)));
            }            
        }
    }
    // int e = getMs();
    // fprintf(stderr, "bfs found, spend %d ms\n", e - s);
    // 如果无法到达目标点，返回空路径
    return path;
}

vector<Point> getPathFromBerthDirMap(int berth_dir_map[berth_num][N][N], int mbx, int mby, int x, int y, int berth_id)
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
    reverse(path.begin(), path.end());
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

int findPosition(vector<Point>& path, int x, int y){
    for(int i = 0; i < path.size(); i++){
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
    for (int i = 1; i <= n; ++i) {
        for (int j = 1; j <= n; ++j) {
            fscanf(file, " %c", &map[i][j]);  // 注意空格，避免读取换行符等
        }
    }

    fclose(file);
}

// left up and leftUp filled
void obstacle_expand(char ch[][N]) {
    for (int i = 1; i <= n; i++) {
        for (int j = 1; j <= n; j++) {
            if (ch[i][j] == '#') {
                if(ch[i - 1][j] == '.') ch[i - 1][j] = '#';
                if(ch[i][j - 1] == '.') ch[i][j - 1] = '#';
                if(ch[i - 1][j - 1] == '.') ch[i - 1][j - 1] = '#';
            }
            else if(ch[i][j] == '*') {
                ch[i - 1][j] = '*';
                ch[i][j - 1] = '*';
                ch[i - 1][j - 1] = '*';
            }
        }
    }
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
            if(n_x <=0 || n_x > n || n_y <= 0 || n_y > n) continue;
            if(grid[n_x][n_y] == '#' || grid[n_x][n_y] == '*') continue;
            if(searched[n_x][n_y]) continue;
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
    
    for(int i=1; i<=n; i++) {
        for(int j=1; j<=n; j++) {
            if(searched[i][j]) continue;
            if(grid[i][j] == '*' || grid[i][j] == '#') continue;
            bfs(grid, connectField, searched, i, j, filed_n);
            filed_n++;
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


void initBerthDirAndDisMap_bfs(int berth_dir_map[berth_num][N][N], int berth_dis_map[berth_num][N][N], char grid[][N], int i, int j, int b_id) {
    queue<Node> que;
    que.push(Node({ i, j }, 0, 0));

    vector<vector<bool>> searched(N, vector<bool>(N, false));
    searched[i][j] = true;
    while (!que.empty()) {
        Node c_p = que.front();
        que.pop();

        berth_dis_map[b_id][c_p.point.x][c_p.point.y] = c_p.g;
        // fprintf(stderr, "dis %d\n", dis);

        for (int d_i = 0; d_i < 4; d_i++) {
            int n_x = c_p.point.x + directions[d_i].x, n_y = c_p.point.y + directions[d_i].y;
            if (n_x <= 0 || n_x > n || n_y <= 0 || n_y > n) continue;
            if (grid[n_x][n_y] == '#' || grid[n_x][n_y] == '*') continue;
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
            que.push(Node({ n_x, n_y }, c_p.g+1, 0));

        }
    }
}

/*
    for each berth, first need to know robot to which point to pull goods, 
    then from this point using bfs search gather direction and set berth_dir_map
*/
void initBerthDirAndDisMap(int berth_dir_map[berth_num][N][N], int berth_dis_map[berth_num][N][N], char grid[][N], vector<Berth>& berth)
{
    for (int i = 0; i < berth_num; i++) {
        initBerthDirAndDisMap_bfs(berth_dir_map, berth_dis_map, grid, berth[i].robot_at_x, berth[i].robot_at_y, i);
    }
}

int countOceanGrid(int x, int y, char grid[][N]) {
    int count = 0;
    for (auto dir : directions) {
        if (grid[x + dir.x][y + dir.y] == '*') count++;
    }
    return count;
}

int countObstacleGrid(int x, int y, char grid[N][N])
{
    int count = 0;
    for (auto dir : directions) {
        if (grid[x + dir.x][y + dir.y] == '#') count++;
    }
    return count;
}

int countNearRobotNum(int x, int y, char grid[N][N], vector<Robot>& robot)
{
    int count = 0;
    for (int i = x - 2; i <= x + 2; ++i) {
        for (int j = y - 2; j <= y + 2; ++j) {
            if (i <= 0 || i > n || j <= 0 || j > n) continue;
            for (int k = 0; k < robot_num; ++k) {
                if (robot[k].x == x && robot[k].y == y) ++count;
            }
        }
    }
    return count;
}

bool judgeRobotAroundIfPass(int x, int y, char grid[N][N])
{
    int left = 0, up = 0, right = 0, down = 0;
    for (int i = x - 1; i <= x + 1; ++i) {
        if (grid[i][y - 1] == '*' || grid[i][y - 1] == '#') ++left;
        if (grid[i][y + 1] == '*' || grid[i][y + 1] == '#') ++right;
    }
    for (int i = y - 1; i <= y + 1; ++i) {
        if (grid[x-1][i] == '*' || grid[x-1][i] == '#') ++up;
        if (grid[x+1][i] == '*' || grid[x+1][i] == '#') ++down;
    }
    return !(((left >= 1 && right >= 1) && (up >= 1 || down >= 1)) || ((left >= 1 || right >= 1) && (up >= 1 && down >= 1)));
}


void fillOcean(char grid[][N], int x1, int y1, int x2, int y2)
{
    for(int i=x1; i<=x2; i++)
        for(int j=y1; j<=y2; j++)
            grid[i][j] = '*';
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

    if (r1_next == r2_next || (r1_next == r2 && r2_next == r1)) {
        // fprintf(stderr, "will collision %d, %d\n", r1_id, r2_id);
        return true;
    }

    return false;
}


void init_robots2eachGDis(int robot_dis_map[robot_num][N][N], char grid[N][N], vector<Robot>& robot) {
    // get all distance from gds to robot
    for (int i = 0; i < robot_num; i++) {
        if (robot[i].taskT == PULL) continue;

        vector<vector<bool>> searched(N, vector<bool>(N, false));
        queue<Node> que;
        que.push(Node({ robot[i].x, robot[i].y }));
        searched[robot[i].x][robot[i].y] = true;

        while (!que.empty()) {
            Node c_p = que.front();
            que.pop();
            robot_dis_map[i][c_p.point.x][c_p.point.y] = c_p.g;

            for (int d_i = 0; d_i < 4; d_i++) {
                int n_x = c_p.point.x + directions[d_i].x, n_y = c_p.point.y + directions[d_i].y;
                if (n_x <= 0 || n_x > n || n_y <= 0 || n_y > n) continue;
                if (grid[n_x][n_y] == '#' || grid[n_x][n_y] == '*'|| searched[n_x][n_y]) continue;
                searched[n_x][n_y] = true;
                que.push(Node({ n_x, n_y }, c_p.g + 1, 0)); // do care h and f
            }
        }
    }
}

// /* test read map and test a* algorithm */
// int main() {
//     char grid[n][n];
//     int connectfield[n][n];
//     readmapfromfile("/home/szt/projects/linuxrelease/maps/map2.txt", grid);
//     obstacle_expand(grid);
//     connectfieldsplit(grid, connectfield);

//     // vector<vector<point>> paths(10);
//     // for(int i=0; i<10; i++) {
//     //     point p = randompoint(n, grid);
//     //     point p2 = randompoint(n, grid);

//     //     paths[i] = astar(grid, p2.x, p2.y, p.x, p.y);
//     // }

//     point p = randompoint(n, grid);
//     while(grid[p.x][p.y] != '.') {
//         p = randompoint(n, grid);
//     }
//     vector<point> path = astar(grid, 30, 32, 33, 68);
//     printf("path size: %d\n", static_cast<int>(path.size()));
//     for (auto p : path) {
//         grid[p.x][p.y] = '+';
//     }

//     for (int i = 1; i <= n; i++) {
//         for (int j = 1; j <= n; j++) {
//             // uninliatiazed shows random num, global variable is default 0
//             printf("%d", connectfield[i][j]); 
//         }
//         printf("\n");
//     }
    
//     return 1;
// }