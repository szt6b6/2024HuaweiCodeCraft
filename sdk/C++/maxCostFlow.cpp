#include "maxCostFlow.h"
using namespace std;

int s,t;
int head[flow_num],cnt;
bool inq[flow_num];
float h[flow_num]; int preDot[flow_num],preEdge[flow_num];
float d[flow_num];
bool vis[flow_num];
int robot_n, task_n;
std::unordered_map<int, int> taskAssignment;

struct edge
{
	int to,next;
    float profit;
    bool flow;
	inline const void create(int u,int v,bool f,float w)
	{
		next=head[u];
		head[u]=cnt;
		flow=f;
		profit=w;
		to=v;
	}
} e[flow_num*flow_num];

inline const void coflow_numect(int u,int v,float w)
{
	e[++cnt].create(u,v,1,w);
	e[++cnt].create(v,u,0,-w);
}

inline const void spfa(float profit[robot_num][flow_num]) //先跑一边spfa确认h数组
{
	fill(inq+1,inq+t+1,0);
	fill(h+1,h+t+1,float(INT_MAX));
	queue<int>q;q.push(s);h[s]=0;inq[s]=1;
	while (q.size())
	{
		int u=q.front();q.pop();
		for (int v,i=head[u];i;i=e[i].next)
			if (e[i].flow&&h[v=e[i].to]>h[u]+e[i].profit)
				if (h[v]=h[u]+e[i].profit,!inq[v])
					inq[v]=1,q.push(v);
	}
}

inline const bool dijkstra(float profit[robot_num][flow_num])
{
	fill(vis+1,vis+t+1,0);
	fill(d+1,d+t+1,float(INT_MAX));
	heap q;q.push(make_pair(d[s]=0,s));
	while (q.size())
	{
		int u=q.top().second;q.pop();
		if (vis[u])continue;vis[u]=1;
		for (int v,i=head[u];i;i=e[i].next)
			if (e[i].flow&&!vis[v=e[i].to]&&d[v]>d[u]+e[i].profit+h[u]-h[v]) //加上势
				preDot[v]=u,preEdge[v]=i,
				q.push(make_pair(d[v]=d[u]+e[i].profit+h[u]-h[v],v));
	}
	return d[t]!=float(INT_MAX);
}

inline const float dinic(float profit[robot_num][flow_num])
{
	float p=0;spfa(profit);
	for (int i=1;i<=cnt;i+=2)e[i].flow=1,e[i+1].flow=0; //还原流量
	for (int i=1;i<=cnt;i++)e[i].profit*=-1; //好像printf的运算顺序是从后往前，所以说它会先运行最大费用，取了一次反，接下来运行最小费用，又会取反回去
	while (dijkstra(profit))
	{
		for (int i=1;i<=t;i++)h[i]+=d[i];
		bool mn=1;
		for (int i=t;i!=s;i=preDot[i])
			mn=min(mn,e[preEdge[i]].flow);
		for (int i=t;i!=s;i=preDot[i]) {
			e[preEdge[i]].flow-=mn,
			e[rev(preEdge[i])].flow+=mn;
			if(i!=t && preDot[i]!=s) {
				if(preDot[i] > 0 && preDot[i] <= robot_n && i > robot_n && i <= robot_n + task_n) {
					taskAssignment[preDot[i]] = i-robot_n;
				}
			}
			
		}
		p+=mn*h[t];
	}
	return p;
}


unordered_map<int, int> getTaskAssignment(int robot_n_, int task_n_, float profit[robot_num][flow_num]) {
	// initialize
	taskAssignment.clear();
	cnt = 0;
	fill(head,head+t+1,0);
	fill(preDot,preDot+t+1,0);
	fill(preEdge,preEdge+t+1,0);
	robot_n = robot_n_;
	task_n = task_n_;
	
	s=robot_n+task_n+1;t=robot_n+task_n+2; //建立超级源点和汇点
	for (int i=1;i<=robot_n;i++)
	 	for (int j=1;j<=task_n;j++) {
            coflow_numect(i,j+robot_n,profit[i-1][j-1]);
        }
	 		
	for (int i=1;i<=robot_n;i++)coflow_numect(s,i,0);
	for (int i=1;i<=task_n;i++)coflow_numect(i+robot_n,t,0);
	float max_profit = -dinic(profit);
	//fprintf(stderr, "max profit: %f\n",-dinic(profit));
	return taskAssignment;
}

// float profit[robot_num][flow_num];
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
// 		printf("robot %d -> task %d\n", it->first, it->second);
// 	}

// 	printf("time: %lld\n", getMs() - start);
// 	return 0;
// }