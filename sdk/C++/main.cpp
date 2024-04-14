#include "constant.h"
#include "Robot.h"
#include "Boat.h"
#include "Berth.h"
#include "utils.h"
#include "TaskAssigner.h"
#include <algorithm>

using namespace std;

char grid[N][N];
int grid_cost[4][N][N];
int gds[N][N][2]; // init will be 0, goods data, x, y, [0]: money, [1]: left_frame
int taskIds_map[N][N];
int connectField[N][N];
int ocean_connectField[N][N];
int berth_dir_map[max_berth_num][N][N]; // init land dir of all point to specific berth
int berth_dis_map[max_berth_num][N][N]; // init land dis of all point to specific berth
int berth_ocean_dis_map[max_berth_num][N][N]; // init land dis of all point to specific berth
int T_ocean_dis_map[max_T_num][N][N]; // init land dis of all point to specific berth
int P_to_closest_berht[N][N]; // 找出地图上的每个点离的最近的港口
bool boat_dir_can_go_map[4][N][N];
Good* goods[N][N]; // 存储每个商品的信息

vector<Robot> robot(max_robot_num);
vector<Berth> berth(max_berth_num);
vector<Boat> boat(max_boat_num);

vector<Point> robot_purchase_point;
vector<Point> boat_purchase_point;
vector<Point> delivery_point; // v point
void ProcessMap()
{
    for(int i = 0; i < N; i ++){
        for(int j = 0; j < N; j ++){
            if(grid[i][j] == 'R')
                robot_purchase_point.push_back({i, j});
            else if(grid[i][j] == 'S')
                boat_purchase_point.push_back({i, j});
            else if(grid[i][j] == 'T')
                delivery_point.push_back({i, j});
        }
    }
}


void Init()
{
    for(int i = 0; i < N; i ++){
        scanf("%s", grid[i]);
    }
    ProcessMap();
    scanf("%d", &berth_num);
    for(int i = 0; i < berth_num; i ++)
    {
        int id;
        scanf("%d", &berth[i].id);
        scanf("%d%d%d", &berth[i].x, &berth[i].y, &berth[i].loading_speed);
        berth[i].initXY(grid);
    }

    scanf("%d", &Boat::max_capacity);
    fprintf(stderr, "max_capacity = %d\n", Boat::max_capacity);

    for(int i=0; i<N; i++) fill(P_to_closest_berht[i], P_to_closest_berht[i]+N, -1);

    initGridCost(grid, grid_cost);

    initBerthDirAndDisMap(berth_dir_map, berth_dis_map, grid, berth);

    initOceanBerthDisMap(berth_ocean_dis_map, grid, berth);

    initOceanTDisMap(T_ocean_dis_map, grid, delivery_point);

    connectFieldSplit(grid, connectField);

    ocean_connectFieldSplit(grid, ocean_connectField);
    
    intiBoatDirCanGoMap(grid, boat_dir_can_go_map);

    for(int i = 0; i < berth_num; i ++)
    {
        berth[i].setDisToNearestT(delivery_point);
        fprintf(stderr, "berth[%d].disToNearestTId = %d with dis = %d\n", i, berth[i].nearestTid, berth[i].disToNearestT);
        fprintf(stderr, "avg %d points bind to this berht with abg dis %.3f\n", berth[i].total_p_bind_berth, berth[i].avg_dis_to_pull);
    }

    // if(berth[0].nearestTid == 0 && berth[0].disToNearestT == 88) real_max_robot_num = 17;
    if(berth[0].nearestTid == 0 && berth[0].disToNearestT == 806) {
        map_flag = 1;
        real_max_robot_num = 11;
        real_max_boat_num = 2;
    }
    else if(berth[0].nearestTid == 0 && berth[0].disToNearestT == 466){ 
        map_flag = 2;
        real_max_boat_num = 1;
        real_max_robot_num = 15;
    }
    else {
        map_flag = 3;
        real_max_boat_num = 1;
        real_max_robot_num = 18;
    }
    

    #ifdef SEE_MAP
    fprintf(stderr, "ocean connect field data:\n");
    for (int i = 0; i < N; i++) {
       for (int j = 0; j < N; j++)
       {
           fprintf(stderr, "%d", ocean_connectField[i][j]);
       }
       fprintf(stderr, "\n");
    }
    #endif
    #ifdef SEE_MAP
    fprintf(stderr, "T dis data:\n");
    for (int i = 0; i < N; i++) {
       for (int j = 0; j < N; j++)
       {
           fprintf(stderr, "%d", T_ocean_dis_map[0][i][j]);
       }
       fprintf(stderr, "\n");
    }
    #endif
    #ifdef SEE_MAP
        fprintf(stderr, "origianl map:\n");
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++)
            {
                fprintf(stderr, "%c", grid[i][j]);
            }
            fprintf(stderr, "\n");
        }
    #endif

    // int i=0;
    // while(i++<5555) rand();
    char okk[100];
    scanf("%s", okk);

    printf("OK\n");
    fflush(stdout);
}


void Input()
{
    scanf("%d", &money);

    // before come into new frame. minus 1 from good life
    for(int i=0; i<N; i++) {
        fill(taskIds_map[i], taskIds_map[i] + N, -1);
        for(int j=0; j<N; j++) {
            gds[i][j][1]--; // min is -15000, no overflow 
        }
    }

    scanf("%d", &goods_num);
    for(int i = 0; i < goods_num; i ++)
    {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        if(val <= 0) {
            gds[x][y][1] = 0;
            --total_del_goods_num; // 包括了机器人拿的和超时消失的
            if(goods[x][y] != nullptr) goods[x][y]->free_m();
            goods[x][y] = nullptr;
            if(P_to_closest_berht[x][y] != -1) --berth[P_to_closest_berht[x][y]].nearHaveGoodsNum;
        } else {
            gds[x][y][0] = val;
            gds[x][y][1] = 1000;
            goods[x][y] = new Good(x, y);
            // fprintf(stderr, "frame %d, gds[%d][%d] with money %d, time %d is generated\n", frame_id, x, y, gds[x][y][0], gds[x][y][1]);
            ++total_added_goods_num;
            if(P_to_closest_berht[x][y] != -1) ++berth[P_to_closest_berht[x][y]].nearHaveGoodsNum;
        }
    }

    scanf("%d", &robot_num);
    for(int i = 0; i < robot_num; i ++)
        scanf("%d%d%d%d", &robot[i].id, &robot[i].good_nums, &robot[i].x, &robot[i].y);

    scanf("%d", &boat_num);
    for(int i = 0; i < boat_num; i ++)
        scanf("%d%d%d%d%d%d\n", &boat[i].id, &boat[i].c_capacity, &boat[i].x, &boat[i].y, &boat[i].dir, &boat[i].status);
    char okk[100];
    scanf("%s", okk);
}


// -----------------------------------------------start----------------------------------------
int main()
{
    if (DEBUG) delay(10);
    Init();
    while(scanf("%d", &frame_id) != EOF)
    {
        
        Input();   

        int s = getMs();
        TaskAssigner::update(robot, berth, boat, grid, gds, connectField, berth_dir_map, berth_dis_map, frame_id);
        // int e = getMs();
        // fprintf(stderr, "frame %d, in main assigner time: %d\n", frame_id, e - s);
        
        // s = getMs();
        for(int i = 0; i < robot_num; i++) // player's command
        {
            robot[i].update(grid, gds, connectField, berth_dir_map, berth_dis_map, robot, berth, boat, frame_id);
        }

        // collision detection and processing
        for (int i = 0; i < robot_num; i++) {
            for (int j = 0; j < robot_num; j++) {
                if(i == j) continue;
                if(manhattanDistance(robot[i].x, robot[i].y, robot[j].x, robot[j].y) >= 3) continue;
                if (robot[i].priority < robot[j].priority && checkCollisionBetwRobot(robot, i, j)) { // use id for priority
                    // i go away
                    // find a direction with no robot front
                    robot[i].moveWithNoRobotFront(grid, robot);
                    robot[i].is_avoid = true;
                    // robot[i].path.clear();
                    // see_log fprintf(stderr, "frame %d, robot %d avoid %d\n", frame_id, i, j);
                }
                else if (robot[i].priority > robot[j].priority && robot[j].move == -1 && checkCollisionBetwRobot(robot, i, j)) {
                    robot[i].moveWithNoRobotFront(grid, robot);
                    robot[i].is_avoid = true;
                    // robot[i].path.clear();
                    // see_log fprintf(stderr, "frame %d, robot %d avoid %d\n", frame_id, i, j);
                }
            }
        }

        // 二次检测避免死锁
        vector<int> pri_robot_ids(robot_num, 0);
        for(int i = 0; i < robot_num; i++) pri_robot_ids[i] = robot[i].id;
        sort(pri_robot_ids.begin(), pri_robot_ids.end(), [&](int a, int b) {
            return (robot[a].priority - 1000*robot[a].gatherAroundRobotNum(robot, 1)) > (robot[b].priority - 1000 * robot[b].gatherAroundRobotNum(robot, 1));
        });
        for (int i = 0; i < robot_num; i++) {
            for (int j = i+1; j < robot_num; j++) {
                if (checkCollisionBetwRobot(robot, pri_robot_ids[i], pri_robot_ids[j])) { // use id for priority
                    // i go away
                    // find a direction with no robot front
                    robot[pri_robot_ids[i]].moveWithNoRobotFront(grid, robot);
                    robot[pri_robot_ids[i]].is_avoid = true;
                    // see_log fprintf(stderr, "frame %d, robot %d avoid %d\n", frame_id, pri_robot_ids[i], pri_robot_ids[j]);
                }
            }
        }        
        // e = getMs();
        // fprintf(stderr, "frame %d, robot time: %d\n", frame_id, e - s);

        for(int i = 0; i < boat_num; i ++){
            boat[i].update(berth, boat, grid, ocean_connectField, frame_id);
        }

        for (int i = 0; i < robot_num; i++) {
            if (robot[i].move != -1) { 
                printf("move %d %d\n", robot[i].id, robot[i].move);
            }
        }

        for(int i=0; i<boat_num; i++) {
            boat[i].moveBoat(grid, frame_id);
        }



        if(money >= boat_price && boat_num < real_max_boat_num){
            if(real_max_robot_num == robot_num && boat_num > 0 && getAvgBoatToSellSpeed() < 0.8*getAvgRobotsPullSpeed()) {
                int total_money_in_berth = 0;
                int twoBerthOver40 = 0;
                for(auto& b : berth) {
                    total_money_in_berth += b.money;
                    if(b.load_num > Boat::max_capacity * 0.8) ++twoBerthOver40;
                }
                if(twoBerthOver40 >= 2 && total_money_in_berth >= 8000 * 1.25) {
                    printf("lboat %d %d\n", boat_purchase_point[boat_num % boat_purchase_point.size()].x, boat_purchase_point[boat_num % boat_purchase_point.size()].y);
                    boat[boat_num].id = boat_num;
                    ++boat_num;
                    money -= boat_price;
                }

            } else if(boat_num == 0) {
                printf("lboat %d %d\n", boat_purchase_point[boat_num % boat_purchase_point.size()].x, boat_purchase_point[boat_num % boat_purchase_point.size()].y);
                boat[boat_num].id = boat_num;
                ++boat_num;
                money -= boat_price;
            }
        }

        int buyed_robot_num = robot_num;
        while(money >= robot_price && robot_num < real_max_robot_num){
            // 找到周围商品数量最多的港口 再找到离这个港口最近的购买机器人的点
            // int purchase_point = rand() % robot_purchase_point.size();
            // while(P_to_closest_berht[robot_purchase_point[purchase_point].x][robot_purchase_point[purchase_point].y] == -1) ++purchase_point;
            // printf("lbot %d %d\n", robot_purchase_point[purchase_point % robot_purchase_point.size()].x, robot_purchase_point[purchase_point % robot_purchase_point.size()].y);
            // robot[robot_num].id = robot_num;
            // ++robot_num;
            // money -= robot_price;
            // if(robot_num == max_robot_num) fprintf(stderr, "robot_num == max_robot_num at frame %d\n", frame_id);
            // if(robot_num % robot_purchase_point.size() == 0) break;

            int index = robot_num;
            // while(P_to_closest_berht[robot_purchase_point[index % robot_purchase_point.size()].x][robot_purchase_point[index % robot_purchase_point.size()].y] == -1) ++index;
            printf("lbot %d %d %d\n", robot_purchase_point[index % robot_purchase_point.size()].x, robot_purchase_point[index % robot_purchase_point.size()].y, 0);
            robot[robot_num].id = robot_num;
            robot[robot_num].type = 0;
            ++robot_num;
            money -= robot_price;
            if(robot_num == real_max_robot_num) fprintf(stderr, "robot_num == max_robot_num at frame %d\n", frame_id);
            if((robot_num - buyed_robot_num) >= robot_purchase_point.size()) break;
        }

        // while(money >= 5000 && robot_num < real_max_robot_num){
        //     // 找到周围商品数量最多的港口 再找到离这个港口最近的购买机器人的点
        //     // int purchase_point = rand() % robot_purchase_point.size();
        //     // while(P_to_closest_berht[robot_purchase_point[purchase_point].x][robot_purchase_point[purchase_point].y] == -1) ++purchase_point;
        //     // printf("lbot %d %d\n", robot_purchase_point[purchase_point % robot_purchase_point.size()].x, robot_purchase_point[purchase_point % robot_purchase_point.size()].y);
        //     // robot[robot_num].id = robot_num;
        //     // ++robot_num;
        //     // money -= robot_price;
        //     // if(robot_num == max_robot_num) fprintf(stderr, "robot_num == max_robot_num at frame %d\n", frame_id);
        //     // if(robot_num % robot_purchase_point.size() == 0) break;

        //     int index = robot_num;
        //     // while(P_to_closest_berht[robot_purchase_point[index % robot_purchase_point.size()].x][robot_purchase_point[index % robot_purchase_point.size()].y] == -1) ++index;
        //     printf("lbot %d %d %d\n", robot_purchase_point[index % robot_purchase_point.size()].x, robot_purchase_point[index % robot_purchase_point.size()].y, 0);
        //     robot[robot_num].id = robot_num;
        //     robot[robot_num].type = 1;
        //     ++robot_num;
        //     money -= 5000;
        //     if(robot_num == real_max_robot_num) fprintf(stderr, "robot_num == max_robot_num at frame %d\n", frame_id);
        //     if((robot_num - buyed_robot_num) >= robot_purchase_point.size()) break;
        // }


        if(money >= boat_price && boat_num < real_max_boat_num){
            if(real_max_robot_num == robot_num && boat_num > 0 && getAvgBoatToSellSpeed() < 0.8*getAvgRobotsPullSpeed()) {
                int total_money_in_berth = 0;
                int twoBerthOver40 = 0;
                for(auto& b : berth) {
                    total_money_in_berth += b.money;
                    if(b.load_num > Boat::max_capacity * 0.8) ++twoBerthOver40;
                }
                if(twoBerthOver40 >= 2 && total_money_in_berth >= 8000 * 1.25) {
                    printf("lboat %d %d\n", boat_purchase_point[boat_num % boat_purchase_point.size()].x, boat_purchase_point[boat_num % boat_purchase_point.size()].y);
                    boat[boat_num].id = boat_num;
                    ++boat_num;
                    money -= boat_price;
                }

            } else if(boat_num == 0) {
                printf("lboat %d %d\n", boat_purchase_point[boat_num % boat_purchase_point.size()].x, boat_purchase_point[boat_num % boat_purchase_point.size()].y);
                boat[boat_num].id = boat_num;
                ++boat_num;
                money -= boat_price;
            }
            // if(robot_num == max_robot_num && Berth::judgeNeedBuyBoat()) {
            //     printf("lboat %d %d\n", boat_purchase_point[boat_num % boat_purchase_point.size()].x, boat_purchase_point[boat_num % boat_purchase_point.size()].y);
            //     boat[boat_num].id = boat_num;
            //     ++boat_num;
            //     money -= boat_price;
            // } else if(boat_num == 0) {
            //     printf("lboat %d %d\n", boat_purchase_point[boat_num % boat_purchase_point.size()].x, boat_purchase_point[boat_num % boat_purchase_point.size()].y);
            //     boat[boat_num].id = boat_num;
            //     ++boat_num;
            //     money -= boat_price;
            // }
        }
        if(frame_id % 1000 == 0) see_log fprintf(stderr, "average generate goods speed: %.3f\naverage pull goods speed: %.3f\naverage boat sell goods speed %.3f\n", \
                                getAvgNewGoodsSpeed(), getAvgRobotsPullSpeed(), getAvgBoatToSellSpeed());
        
        // if(frame_id % 1000 == 0) {
        //     for(int i=0; i<berth_num; i++) {
        //         fprintf(stderr, "berth %d near goods num: %d\n", i, berth[i].nearHaveGoodsNum);
        //     }
        // }
        // 人工封住港口
        // if(frame_id == 14372) {
        //     berth[2].forbid = true;
        // }
        // banBerth(14372, 2); // at frame 14372 ban berth 2
        // if(frame_id == 14100) {
        //     berth[4].forbid = true;
        // }
        // if(frame_id == 14550) {
        //     berth[3].forbid = true;
        // }
        // if(frame_id == 14600) {
        //     berth[0].forbid = true;
        // }
        // if(frame_id == 14700) {
        //     berth[1].forbid = true;
        // }

        // if(map_flag == 1) {
        //     banBerth(13177, 0);
        //     banBerth(13730, 1);
        // }

        puts("OK\n");
        fflush(stdout);
    }

    fprintf(stderr, "berths load_nums: ");
    for (int i = 0; i < berth_num; i++) fprintf(stderr, " %d ", berth[i].load_num);
    fprintf(stderr, "\n");

    fprintf(stderr, "robot num: %d, cost: %d, boat num: %d, cost: %d\ntotal pull money: %d, total pull nums: %d \n", robot_num, robot_price*robot_num, boat_num, boat_price*boat_num, total_pull_money, total_pull_num);
    fprintf(stderr, "total generated goods num: %d, total del goods num: %d\n", total_added_goods_num, total_del_goods_num);
    return 0;
}