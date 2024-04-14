#include "TaskAssigner.h"
#include "maxCostFlow.h"
#include "math.h"

extern int taskIds_map[N][N];
extern int P_to_closest_berht[N][N];
extern Good* goods[N][N];

vector<Task> TaskAssigner::taskQueue = vector<Task>(N*N);
int total_robots = robot_num;
int total_tasks = N*N;

double profit[max_robot_num][flow_num];  // 任务与执行者的收益矩阵, 用于最大流最大收益

int robot_dis_map[max_robot_num][N][N]; // 记录机器人到地图上每个点的距离

// later change this，idea robot look around circle by circle
void TaskAssigner::update(vector<Robot> &robot, vector<Berth> &berth, vector<Boat> &boat,
                            char grid[][N], int gds[][N][2], int connectField[][N], int berth_dir_map[max_berth_num][N][N], int berth_dis_map[max_berth_num][N][N], int frame)
{
    vector<int> r_ids;
    for(int i=0; i<robot_num; i++) {
        if(robot[i].taskT == FREE || robot[i].taskT == GET) r_ids.push_back(i);
    }
    if (r_ids.size() <= 0) return; // no robot free, no need update task queue
    total_robots = r_ids.size();
GATHER_TASK:

    int t_i = 0;
    for(int i=0; i<N; i++) {
        fill(taskIds_map[i], taskIds_map[i]+N, -1);
        for(int j=0; j<N; j++) {
            if(gds[i][j][1] > 0) {
                // int berth_i = P_to_closest_berht[i][j]; // map2 use this only
                // if(frame > 13500) berth_i = findBerthToPULL(berth, boat, berth_dis_map, connectField, i, j, frame);
                int berth_i = findBerthToPULL(berth, boat, berth_dis_map, connectField, i, j, frame);
                if(berth_i == -1 || berth[berth_i].nearestTid == -1) continue;
                taskQueue[t_i++] = Task(frame + gds[i][j][1], i, j, berth[berth_i].x, berth[berth_i].y, gds[i][j][0], berth_i, berth_dis_map[berth_i][i][j]);
            }
        }
    }
    total_tasks = t_i;

ASSIGN_TASK:
    // 采用最大流最大收益算法
    for (int i = 0; i < robot_num; i++) {
        fill(profit[i], profit[i] + flow_num, -N*N);
        for (int j = 0; j < N; j++) {
            fill(robot_dis_map[i][j], robot_dis_map[i][j] + N, 1);
        }
    }

    for (int i=0; i<r_ids.size(); i++) {
        int r_id = r_ids[i];

        for (int j = 0; j < t_i; j++) {
            Task& t = taskQueue[j];
            if (connectField[robot[r_id].x][robot[r_id].y] != connectField[t.b_x][t.b_y]) {
                profit[i][j] = -N*N*N;
                continue;
            };
            int robot_to_g_dis = goods[t.b_x][t.b_y]->getDis(robot[r_id].x, robot[r_id].y);
            int gap_frame = t.deadline - frame - robot_to_g_dis;
            if (gap_frame <= 0) {
                profit[i][j] = -N*N*N;
                continue;
            }

            // float gap_weight = (gap_frame+250) / 1250.0;
            // profit[i][j] = 1250.0 / (gap_frame + 350) * (t.money * 1.0 / (robot_to_g_dis + 0.3*t.to_dest_berth_dis));
            //profit[i][j] = 1000 + 2*t.money - (robot_to_g_dis + 0.058 * gap_frame);
            //92806
            // profit[i][j] = t.money*t.money  / (robot_to_g_dis + 0.50 * t.to_dest_berth_dis) / (0.05 * gap_frame + 90.0);
            // profit[i][j] = (1250.0 / (gap_frame + 350) + 1) * (t.money*1.0) / (robot_to_g_dis + 0.3*t.to_dest_berth_dis);
            //profit[i][j] = 1250.0 / (gap_frame + 350) * t.money * 1.0 / (robot_to_g_dis + 0.3 * t.to_dest_berth_dis);
    
            //profit[i][j] = -log(gap_frame + robot_to_g_dis + 100) + 1.3*log(t.money) - log(robot_to_g_dis + 0.3 * t.to_dest_berth_dis);
            
            // 903 918 983
            profit[i][j] = -log(gap_frame + robot_to_g_dis + 250) + 1.5*log(t.money) - log(robot_to_g_dis + 0.2 * t.to_dest_berth_dis);

            // 914 887 957
            // profit[i][j] = -log(gap_frame + robot_to_g_dis + Berth::getMeanDisToGds()*1.0) + 1.3*log(t.money) - log(robot_to_g_dis + 0.25 * t.to_dest_berth_dis);

            // 902 867 977
            // profit[i][j] = -log(gap_frame + robot_to_g_dis + Berth::getMeanDisToGds()*1.1) + 1.0*log(t.money) - log(robot_to_g_dis + 0.3 * t.to_dest_berth_dis);
            // profit[i][j] = -log(gap_frame + robot_to_g_dis + Berth::getMeanDisToGds()*2.0) + 1.2*log(t.money) - log(robot_to_g_dis + 0.25 * t.to_dest_berth_dis);
            // fprintf(stderr, "dis %d, money %d, profit[%d][%d] = %f\n", robot_to_g_dis, t.money, i, j, profit[i][j]);
        }
    }

    unordered_map<int, int> assigned_res = getTaskAssignment(total_robots, total_tasks+total_robots, profit); // 额外机器人个数个空闲任务 价值为0
    // 分配任务
    // fprintf(stderr, "frame %d, assigned_res size = %d\n", frame, assigned_res.size());
    for (auto& it : assigned_res) {
        int r_id = r_ids[it.first-1];
        int t_id = it.second-1;
        if (t_id >= t_i) { robot[r_id].taskT = FREE; continue; }
        Task& t = taskQueue[t_id];
        robot[r_id].setBSxy(t.b_x, t.b_y, t.s_x, t.s_y);
        robot[r_id].taskT = GET;// robot turn to GET
        robot[r_id].get_deadline = t.deadline;
        robot[r_id].dst_pull_berth_id = t.dest_bert_id;
        // fprintf(stderr, "robot %d bind task %d, dst [%d, %d]\n", r_id, t_id, t.b_x, t.b_y);
    }
}

// need to find x, y; which is a place to go, goods assign to berth
int TaskAssigner::findBerthToPULL(vector<Berth> &berth, vector<Boat> &boat, int berth_dis_map[max_berth_num][N][N], int connectField[][N], int x, int y, int frame)
{
    double e_profit = double(INT_MIN);
    int select = -1;
    for(int i=0; i<berth_num; i++) {
        // find nearst and in same connecField
        if(connectField[x][y] != connectField[berth[i].x][berth[i].y] || berth[i].nearestTid == -1) continue;

        if(berth[i].forbid) continue;

        double profit = -1.0 * berth_dis_map[i][x][y];

        if (e_profit < profit) {
            select = i;
            e_profit = profit;
        }
    }
    return select;
}