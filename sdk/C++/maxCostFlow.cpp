#include "maxCostFlow.h"
using namespace std;

int start, target;
int pre[flow_num], cnt;
bool inque[flow_num];
double h_dis[flow_num];
int preDot[flow_num], preEdge[flow_num];
double d_dis[flow_num];
bool e_visited[flow_num];
int robot_n, task_n;
std::unordered_map<int, int> taskAssignment;

struct edge
{
	int to, next;
	double profit;
	bool flow;
	inline const void create(int u, int v, bool f, double w)
	{
		next = pre[u];
		pre[u] = cnt;
		flow = f;
		profit = w;
		to = v;
	}
} edges[flow_num * flow_num];

void connect_graph(int u, int v, double w)
{
	edges[++cnt].create(u, v, 1, w);
	edges[++cnt].create(v, u, 0, -w);
}

void spfa(double profit[max_robot_num][flow_num])
{
	fill(inque + 1, inque + target + 1, 0);
	fill(h_dis + 1, h_dis + target + 1, double(INT_MAX));
	queue<int> q;
	q.push(start);
	h_dis[start] = 0;
	inque[start] = 1;
	while (q.size())
	{
		int u = q.front();
		q.pop();
		for (int v, i = pre[u]; i; i = edges[i].next)
			if (edges[i].flow && h_dis[v = edges[i].to] > h_dis[u] + edges[i].profit)
				if (h_dis[v] = h_dis[u] + edges[i].profit, !inque[v])
					inque[v] = 1, q.push(v);
	}
}

bool dijkstra(double profit[max_robot_num][flow_num])
{
	fill(e_visited + 1, e_visited + target + 1, false);
	fill(d_dis + 1, d_dis + target + 1, double(INT_MAX));
	heap q;
	q.push(make_pair(d_dis[start] = 0, start));
	while (q.size())
	{
		int u = q.top().second;
		q.pop();
		if (e_visited[u])
			continue;
		e_visited[u] = true;
		for (int v, i = pre[u]; i; i = edges[i].next)
			if (edges[i].flow && !e_visited[v = edges[i].to] && d_dis[v] > d_dis[u] + edges[i].profit + h_dis[u] - h_dis[v]) // 加上势
				preDot[v] = u, preEdge[v] = i,
				q.push(make_pair(d_dis[v] = d_dis[u] + edges[i].profit + h_dis[u] - h_dis[v], v));
	}
	return d_dis[target] != double(INT_MAX);
}

double dinic(double profit[max_robot_num][flow_num])
{
	double p = 0;
	spfa(profit);
	for (int i = 1; i <= cnt; i += 2)
		edges[i].flow = 1, edges[i + 1].flow = 0;
	for (int i = 1; i <= cnt; i++)
		edges[i].profit *= -1;
	while (dijkstra(profit))
	{
		for (int i = 1; i <= target; i++)
			h_dis[i] += d_dis[i];
		bool mn = 1;
		for (int i = target; i != start; i = preDot[i])
			mn = min(mn, edges[preEdge[i]].flow);
		for (int i = target; i != start; i = preDot[i])
		{
			edges[preEdge[i]].flow -= mn,
				edges[rev(preEdge[i])].flow += mn;
			if (i != target && preDot[i] != start)
			{
				if (preDot[i] > 0 && preDot[i] <= robot_n && i > robot_n && i <= robot_n + task_n)
				{
					taskAssignment[preDot[i]] = i - robot_n;
				}
			}
		}
		p += mn * h_dis[target];
	}
	return p;
}

unordered_map<int, int> getTaskAssignment(int robot_n_, int task_n_, double profit[max_robot_num][flow_num])
{
	// initialize
	taskAssignment.clear();
	cnt = 0;
	fill(pre, pre + target + 1, 0);
	fill(preDot, preDot + target + 1, 0);
	fill(preEdge, preEdge + target + 1, 0);
	robot_n = robot_n_;
	task_n = task_n_;

	start = robot_n + task_n + 1;
	target = robot_n + task_n + 2; // 建立源点和汇点
	for (int i = 1; i <= robot_n; i++)
		for (int j = 1; j <= task_n; j++)
		{
			connect_graph(i, j + robot_n, profit[i - 1][j - 1]);
		}

	for (int i = 1; i <= robot_n; i++)
		connect_graph(start, i, 0);
	for (int i = 1; i <= task_n; i++)
		connect_graph(i + robot_n, target, 0);
	double max_profit = -dinic(profit);
	// fprintf(stderr, "max profit: %f\n",-dinic(profit));
	return taskAssignment;
}

// double profit[robot_num][flow_num];
// int main()
// {
// 	long long start = getMs();
// 	for(int i=0; i<robot_num; i++) {
// 		for(int j=0; j<200; j++) {
// 			profit[i][j] = rand() % 100 + 1;
// 			printf("%.0f ", profit[i][j]);
// 		}
// 		printf("\n");
// 	}
// 	taskAssignment = getTaskAssignment(10, 200, profit);

// 	for(auto it = taskAssignment.begin(); it != taskAssignment.end(); it++) {
// 		printf("robot %d_dis -> task %d_dis\n", it->first, it->second);
// 	}

// 	printf("time: %lld\n", getMs() - start);
// 	return 0;
// }