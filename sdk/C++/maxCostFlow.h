#ifndef MAXCOSTFLOW_H
#define MAXCOSTFLOW_H

#include "utils.h"
#include <algorithm>
#include <vector>
#include <unordered_map>
#define rev(x) x%2?x+1:x-1

typedef std::pair<int,int> node;
typedef std::priority_queue<node,std::vector<node>,std::greater<node>> heap;

std::unordered_map<int, int> getTaskAssignment(int robot_n, int task_n, double profit[max_robot_num][flow_num]);

#endif