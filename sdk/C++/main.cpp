#include <stdio.h>
#include <string.h>
#include "constant.h"
#include "Berth.h"
#include "utils.h"
#include "Robot.h"
#include "Boat.h"
#include "TaskAssigner.h"
#include <cassert>
#include <algorithm>

using namespace std;

int money, id;
char grid[N][N]; // map data
int gds[N][N][2]; // init will be 0, goods data, x, y, [0]: money, [1]: left_frame
bool gds_locked[N][N]; // indicate one robot are go to get gds[i][j]
int taskIds_map[N][N];
int connectField[N][N];
int berth_dir_map[berth_num][N][N]; // init dir of all point to specific berth
int berth_dis_map[berth_num][N][N]; // init dis of all point to specific berth
vector<Robot> robot(robot_num);
vector<Berth> berth(berth_num);
vector<Boat> boat(boat_num);

bool map1 = false;

void Init()
{

    for(int i = 1; i <= n; i ++) // get 200 lines, map data
        scanf("%s", grid[i] + 1);

    #ifdef SEE_MAP  
    fprintf(stderr, "original map data:\n");
    for(int i=1; i<=n; i++) {
        for(int j=1; j<=n; j++) 
        {
            fprintf(stderr, "%c", grid[i][j]);
        }
        fprintf(stderr, "\n");
    }
    fprintf(stderr, "\n");
    #endif


    for(int i = 0; i < berth_num; i ++) // get 10 lines, berth data
    {
        int id;
        scanf("%d", &id);
        berth[id].id = id;
        scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
        fprintf(stderr, "berth %d: %d %d %d %d\n", id, berth[id].x, berth[id].y, berth[id].transport_time, berth[id].loading_speed);
        // init farest and nearest berth
        if(Berth::max_transport_time < berth[id].transport_time) {
            Berth::max_transport_time = berth[id].transport_time;
            Berth::farest_berth_id = id;
        }
        if(Berth::min_transport_time > berth[id].transport_time) {
            Berth::min_transport_time = berth[id].transport_time;
            Berth::nearest_berth_id = id;
        }
        berth[id].x += 1;
        berth[id].y += 1;
        berth[id].initXY(grid);
    }
    // if (berth[9].x == 150+1 && berth[9].y == 109+1) { map1 = true; fprintf(stderr, "................map1............ \n");}
    Berth::average_transport_time = (Berth::max_transport_time + Berth::min_transport_time) / 2;
    fprintf(stderr, "max_transport_time = %d, min_transport_time = %d, average_transport_time = %d\n", Berth::max_transport_time, Berth::min_transport_time, Berth::average_transport_time);

    for (int i = 0; i < robot_num; i++) {
        for (int j = 0; j < N; j++) fill(berth_dis_map[i][j], berth_dis_map[i][j] + N, INT_MAX);
    }

    initBerthDirAndDisMap(berth_dir_map, berth_dis_map, grid, berth);
    for(int k=0; k<berth_num; k++) {
        int count = 0;
        for(int i=1; i<N; i++) {
            for(int j=1; j<N; j++) {
                if(berth_dis_map[k][i][j] != INT_MAX) {
                    berth[k].average_dis2eachPoint += berth_dis_map[k][i][j];
                    ++count;
                }
            }
        }
    }

    #ifdef SEE_MAP
    fprintf(stderr, "berth_dir_map 4 data:\n");
    for (int i = 1; i <= n; i++) {
        for (int j = 1; j <= n; j++)
        {
            fprintf(stderr, "%d", berth_dir_map[4][i][j]);
        }
        fprintf(stderr, "\n");
    }
    #endif

    #ifdef SEE_MAP
    fprintf(stderr, "after process berth data:\n");
    for(int i=1; i<=n; i++) {
        for(int j=1; j<=n; j++) 
        {
            fprintf(stderr, "%c", grid[i][j]);
        }
        fprintf(stderr, "\n");
    }
    #endif
    connectFieldSplit(grid, connectField);

    #ifdef SEE_MAP
    fprintf(stderr, "connect field data:\n");
    for(int i=1; i<=n; i++) {
        for(int j=1; j<=n; j++) 
        {
            fprintf(stderr, "%d", connectField[i][j]);
        }
        fprintf(stderr, "\n");
    }
    #endif

    scanf("%d", &Boat::max_capacity); // get boat capacity
    fprintf(stderr, "max capacity: %d\n", Boat::max_capacity);

    char okk[100];
    scanf("%s", okk); // get OK
    printf("OK\n"); // outtut OK means the initialization is done
    fflush(stdout);
}

int Input()
{
    scanf("%d%d", &id, &money); // get current frame id and money
    int num;
    scanf("%d", &num); // get added goods number

    // before come into new frame. minus 1 from good life
    for(int i=0; i<N; i++) {
        fill(taskIds_map[i], taskIds_map[i] + N, -1);
        for(int j=0; j<N; j++) {
            gds[i][j][1]--; // min is -15000, no overflow 
        }
    }

    for(int i = 1; i <= num; i ++) // get added goods data
    {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        x += 1; y += 1;
        gds[x][y][0] = val;
        gds[x][y][1] = 1000; // 1000 frames to disappear
    }
    for(int i = 0; i < robot_num; i ++) // get robot data
    {
        scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y, &robot[i].status);
        robot[i].x += 1;
        robot[i].y += 1;
    }

    for (int i = 0; i < boat_num; i++) {// get boat data
        scanf("%d%d\n", &boat[i].status, &boat[i].target_pos);
    }

    char okk[100];
    scanf("%s", okk); // get OK
    return id;
} 

int main(int argc, char* argv[])
{
    if(DEBUG) delay(10);

    // assert(argc == 14);

    // berth_dismap_weight = stof(argv[1]);
    // berth_loading_weight = stof(argv[2]);
    // berth_averageDis_weight = stof(argv[3]);
    // berth_haveBoatNum_weight = stof(argv[4]);
    // berth_haveRnum_weight = stof(argv[5]);

    // robot_moneyDis_weight = stof(argv[6]);
    // robot_avgDis_weight = stof(argv[7]);
    // robot_haveRnum_weight = stof(argv[8]);
    // robot_haveBoat_weight = stof(argv[9]);

    // boat_loadNum_weight = stof(argv[10]);
    // boat_robotNum_weight = stof(argv[11]);
    // boat_avgDis_weight = stof(argv[12]);
    // boat_transTime_weight = stof(argv[13]);

    Init();
    for(int frame = 1; frame <= 15000; frame++)
    {
        int id = Input(); // get data from judger 

        long long start = getMs();

        TaskAssigner::update(robot, berth, boat, grid, gds, gds_locked, connectField, berth_dir_map, berth_dis_map, frame);
        
        for(int i = 0; i < robot_num; i++) // player's command
        {
            robot[i].update(grid, gds, gds_locked, connectField, berth_dir_map, berth_dis_map, robot, berth, boat, frame);
            LOG_see_robot(frame, i, robot[i].taskT, robot[i].money, robot[i].dst_pull_berth_id, robot[i].mbx, robot[i].mby, robot[i].msx, robot[i].msy, robot[i].x, robot[i].y);
        }

        // collision detection and processing
        for (int i = 0; i < robot_num; i++) {
            for (int j = 0; j < robot_num; j++) {
                if (robot[i].priority < robot[j].priority && checkCollisionBetwRobot(robot, i, j)) { // use id for priority
                    // i go away
                    // find a direction with no robot front
                    robot[i].moveWithNoRobotFront(grid, robot);
                    robot[i].path.clear();
                    robot[i].is_avoid = true;
                    fprintf(stderr, "frame %d, robot %d avoid %d\n", frame, i, j);
                }
                else if (robot[i].priority > robot[j].priority && robot[j].move == -1 && checkCollisionBetwRobot(robot, i, j)) {
                    robot[i].moveWithNoRobotFront(grid, robot);
                    robot[i].path.clear();
                    robot[i].is_avoid = true;
                    fprintf(stderr, "frame %d, robot %d avoid %d\n", frame, i, j);
                }
            }
        }

        // 二次检测避免死锁
        vector<int> pri_robot_ids(robot_num, 0);
        for(int i = 0; i < robot_num; i++) pri_robot_ids[i] = robot[i].id;
        sort(pri_robot_ids.begin(), pri_robot_ids.end(), [&](int a, int b) {
            return robot[a].priority - (robot[a].move==-1)*100000 > robot[b].priority-(robot[b].move==-1)*100000;
        });
        for (int i = 0; i < robot_num; i++) {
            for (int j = i+1; j < robot_num; j++) {
                if (checkCollisionBetwRobot(robot, pri_robot_ids[i], pri_robot_ids[j])) { // use id for priority
                    // i go away
                    // find a direction with no robot front
                    robot[pri_robot_ids[i]].moveWithNoRobotFront(grid, robot);
                    robot[pri_robot_ids[i]].path.clear();
                    robot[pri_robot_ids[i]].is_avoid = true;
                    fprintf(stderr, "frame %d, robot %d avoid %d\n", frame, pri_robot_ids[i], pri_robot_ids[i]);
                }
            }
        }


        for (int i = 0; i < robot_num; i++) {
            if (robot[i].move != -1) { 
                printf("move %d %d\n", robot[i].id, robot[i].move);
                LOG_see_rmove(frame, robot[i].id, directions_s[robot[i].move]);
            }
        }

        for (int i = 0; i < boat_num; i++) {
            boat[i].update(berth, boat, frame);
        }

        // robot: move id, 0/1/2/3, get id, pull id; 
        //// berth: ship id, 0-9, go id
        long long end = getMs();
        int time = end - start;
        LOG_INFO(frame, time, time spend ms);

        puts("OK");
        fflush(stdout);
    }
    fprintf(stderr, "total load num: %d, total pull money: %d, current money: %d\n", total_load_num, total_pull_money, money);
    return 0;
}
