#include "TaskAssigner.h"

extern int taskIds_map[N][N];

vector<Task> taskQueue(berth_num*N*N);
int total_robots = robot_num;
int total_tasks = flow_num;

float profit[robot_num][flow_num];  // 任务与执行者的收益矩阵, 用于最大流最大收益

int robot_dis_map[robot_num][N][N]; // 记录机器人到地图上每个点的距离

void TaskAssigner::update(vector<Robot> &robot, vector<Berth> &berth, vector<Boat> &boat,
                            char grid[][N], int gds[][N][2], bool gds_locked[][N], 
                            int connectField[][N], int berth_dir_map[berth_num][N][N], int berth_dis_map[berth_num][N][N], int frame)
{
    vector<int> r_ids;
    for(int i=0; i<robot_num; i++) {
        if(robot[i].taskT == FREE || robot[i].taskT == GET) r_ids.push_back(i);
    }
    if (r_ids.size() <= 0) return; // no robot free, no need update task queue
    total_robots = r_ids.size();

GATHER_TASK:

    // 收集每个商品价值和其到每个港口的距离 组成所有的pull队列
    int t_i = 0;
    for(int i=1; i<=n; i++) {
        for(int j=1; j<=n; j++) {
            if(gds[i][j][1] > 0) {
                int berth_i = findBerthToPULL(berth, boat, berth_dis_map, connectField, i, j, frame);
                if(berth_i != -1) 
                    taskQueue[t_i++] = Task(frame + gds[i][j][1], i, j, berth[berth_i].robot_at_x, berth[berth_i].robot_at_y, gds[i][j][0], berth_i, berth_dis_map[berth_i][i][j]);
            }
        }
    }
    total_tasks = t_i;

ASSIGN_TASK:

    // 采用最大流最大收益算法
    for (int i = 0; i < robot_num; i++) {
        fill(profit[i], profit[i] + flow_num, 0.0);
        for (int j = 0; j < N; j++) {
            fill(robot_dis_map[i][j], robot_dis_map[i][j] + N, N*N);
        }
    }
    init_robots2eachGDis(robot_dis_map, grid, robot);

    for (int i=0; i<r_ids.size(); i++) {
        int r_id = r_ids[i];

        for (int j = 0; j < t_i; j++) {
            Task& t = taskQueue[j];
            if (connectField[robot[r_id].x][robot[r_id].y] != connectField[t.b_x][t.b_y]) {
                profit[i][j] = -1;
                continue;
            };
            int robot_to_g_dis = robot_dis_map[r_id][t.b_x][t.b_y];
            int gap_frame = t.deadline - frame - robot_to_g_dis;
            if (gap_frame <= 0) {
                profit[i][j] = -1;
                continue;
            }
            profit[i][j] = 10.0 * t.money / (robot_to_g_dis + 0.058 * gap_frame);

            // RobotTaskProfit rela(r_id, j, t.money / (robot_to_g_dis + 0.25 * t.to_dest_berth_dis + 0.058 * gap_frame)); // 0.05这个值不一样 不同的图差很大
        }
    }

    unordered_map<int, int> assigned_res = getTaskAssignment(total_robots, total_tasks+total_robots, profit); // 额外机器人个数个空闲任务 价值为0
    // 分配任务
    for (auto& it : assigned_res) {
        int r_id = r_ids[it.first-1];
        int t_id = it.second-1;
        if (t_id >= total_tasks) continue; // 设为空闲状态
        Task& t = taskQueue[t_id];
        robot[r_id].setBSxy(t.b_x, t.b_y, t.s_x, t.s_y);
        robot[r_id].taskT = GET;// robot turn to GET
        robot[r_id].get_deadline = t.deadline;
        robot[r_id].dst_pull_berth_id = t.dest_bert_id;
    }

/*
    priority_queue<RobotTaskProfit, vector<RobotTaskProfit>, greater<RobotTaskProfit>> taskBindHeap;

    vector<vector<bool>> goodsAssigned(N, vector<bool>(N, false));

    for (int i = 0; i < robot_num; i++) {
        for (int j = 0; j < N; j++) fill(robot_dis_map[i][j], robot_dis_map[i][j] + N, N*N);
    }
    init_robots2eachGDis(robot_dis_map, grid, robot);

    // 根据已有关系构造优先级堆
    for (auto iter = r_ids.begin(); iter != r_ids.end(); iter++) {
        int r_id = *iter;

        for (int i = 0; i < t_i; i++) {
            Task& t = taskQueue[i];
            if (connectField[robot[r_id].x][robot[r_id].y] != connectField[t.b_x][t.b_y]) continue;
            int robot_to_g_dis = robot_dis_map[r_id][t.b_x][t.b_y];
            int gap_frame = t.deadline - frame - robot_to_g_dis;
            if (gap_frame <= 0) continue;

            RobotTaskProfit rela(r_id, i, t.money / (robot_to_g_dis + 0.25 * t.to_dest_berth_dis + 0.058 * gap_frame)); // 0.05这个值不一样 不同的图差很大
            taskBindHeap.push(rela);
        }
    }

    // 从构造好的堆中 去任务分配给机器人
    while (!taskBindHeap.empty() && !r_ids.empty()) {
        int r_id = taskBindHeap.top().r_id;
        int t_id = taskBindHeap.top().t_id;
        Task& t = taskQueue[t_id];

        if (r_ids.find(r_id) != r_ids.end() && !goodsAssigned[t.b_x][t.b_y]) {
            // assign task
            r_ids.erase(r_id);
            robot[r_id].setBSxy(t.b_x, t.b_y, t.s_x, t.s_y);
            robot[r_id].taskT = GET;// robot turn to GET
            robot[r_id].get_deadline = t.deadline;
            robot[r_id].dst_pull_berth_id = t.dest_bert_id;
            goodsAssigned[t.b_x][t.b_y] = true;
        }
        taskBindHeap.pop();
    }
*/

}

void TaskAssigner::updateForOneRobot(vector<Robot> &robot, vector<Berth> &berth, vector<Boat> &boat, char grid[][N], int gds[][N][2], 
                                        bool gds_locked[][N], int connectField[][N], int berth_dir_map[berth_num][N][N], 
                                        int berth_dis_map[berth_num][N][N], int frame, int r_i)
{
GATHER_TASK:

    int t_i = 0;
    for(int i=1; i<=n; i++) {
        for(int j=1; j<=n; j++) {
            if(gds[i][j][1] > 0 && !gds_locked[i][j]) { 
                int berth_i = findBerthToPULL(berth, boat, berth_dis_map, connectField, i, j, frame);
                if(berth_i == -1) continue;
                Task t(frame+ gds[i][j][1], i, j, berth[berth_i].robot_at_x, berth[berth_i].robot_at_y, gds[i][j][0], berth_i, berth_dis_map[berth_i][i][j]);
                // LOG_see_position(frame, 0, t->b_x, t->b_y, t->s_x, t->s_y);
                taskQueue[t_i++] = t;
            }
        }
    }

ASSIGN_TASK:
    
    Task t;
    LOG_see_task(frame, r_i, t_i);
    double e_profit = double(INT_MIN);
    for(int j=0; j<t_i; j++) {
        Task task = taskQueue[j];

        if(gds_locked[task.b_x][task.b_y]) continue;
        if(connectField[robot[r_i].x][robot[r_i].y] != connectField[task.b_x][task.b_y]) continue;

        int disToGet = 0;
        if(robot[r_i].after_pull_at_berth_id != -1 && grid[robot[r_i].x][robot[r_i].y] == 'B')
            disToGet = berth_dis_map[robot[r_i].after_pull_at_berth_id][task.b_x][task.b_y];
        else
            disToGet = manhattanDistance(robot[r_i].x, robot[r_i].y, task.b_x, task.b_y); // later improve

        if(frame + disToGet + 1 > task.deadline) continue; // good disappear befor robot arrive
        int disToPut = berth_dis_map[task.dest_bert_id][task.b_x][task.b_y];

        // float profit = task.money / (disToGet + disToPut + 0.01) - berth[task.dest_bert_id].average_dis2eachPoint;
        if (frame > 15000 - 2 * min(Berth::max_transport_time, Berth::min_transport_time + berth2berth_dis)) robot_haveBoat_weight = 1000;
        double profit = robot_moneyDis_weight * task.money / (disToGet + 0.01) +
                        robot_avgDis_weight * berth[task.dest_bert_id].average_dis2eachPoint +
                        robot_haveRnum_weight * berth[task.dest_bert_id].r_num +
                        robot_haveBoat_weight * berth[task.dest_bert_id].boat_num;
        // fprintf(stderr, "money = %d, dis = %d\n", task.money, disToGet + disToPut);
        // later consider more elements
        if(e_profit < profit) {
            t = task;
            e_profit = profit;
        }
    }
    if(e_profit != double(INT_MIN)) {
        robot[r_i].setBSxy(t.b_x, t.b_y, t.s_x, t.s_y); 
        robot[r_i].taskT = GET;// robot turn to GET
        robot[r_i].get_deadline = t.deadline;
        robot[r_i].dst_pull_berth_id = t.dest_bert_id;
        gds_locked[t.b_x][t.b_y] = true; // + lock
        LOG_see_position(r_i, r_i, t.b_x, t.b_y, t.s_x, t.s_y);
    }
}

// need to find x, y; which is a place to go, goods assign to berth
int TaskAssigner::findBerthToPULL(vector<Berth> &berth, vector<Boat> &boat, int berth_dis_map[berth_num][N][N], int connectField[][N], int x, int y, int frame)
{
    float e_profit = float(INT_MIN);
    int select = -1;
    for(int i=0; i<berth_num; i++) {
        // find nearst and in same connecField
        if(connectField[x][y] != connectField[berth[i].robot_at_x][berth[i].robot_at_y]) continue;

        float punish_time_not_enough = 0;
        if(berth[i].boat_num == 0 && frame > 15000 - 2 *  min(berth[i].transport_time, Berth::min_transport_time + berth2berth_dis)) punish_time_not_enough = -100000.0;

        float timeWeight = 0;
        if(frame < 15000 - min(Berth::max_transport_time, Berth::min_transport_time + berth2berth_dis) * 2) timeWeight = 0;
        else timeWeight = 1;
        float profit = berth_dismap_weight * berth_dis_map[i][x][y] +
            timeWeight * 40000.0 / berth[i].transport_time +
            timeWeight * 10000 * berth[i].boat_num + punish_time_not_enough;

        // fprintf(stderr, "frame %d, align task, berth %d, goodToBerthDis %d\n", frame, i, goodToBerthDis);
        if (e_profit < profit) {
            select = i;
            e_profit = profit;
            // fprintf(stderr, "frame %d, align task, berth %d, selected goodToBerthDis %d\n", frame, i, dis);
        }
    }
    return select;
}