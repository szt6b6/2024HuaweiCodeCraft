#include "Boat.h"
#include "utils.h"
#include <climits>

int Boat::max_capacity = 0;
int Boat::i = 0;
extern int money;

void Boat::update(vector<Berth>& berth, vector<Boat>& boat, int frame)
{

	transTime--;
	if(status == 0) return; // moveing  or wait do not make logic process
	if(status == 2 && !forward_flag) return;

LOAD:
	if (taskT == LOADING) {
		
		// judge: in loading, and frame + transport_time >= 14990, let timeout_flag = true
		if(frame + min(berth[go_berth_id].transport_time, Berth::min_transport_time + berth2berth_dis) >= 14999) {
			taskT = TO_SELL;
			berth[go_berth_id].current_boat_id = -1;
			if(berth[go_berth_id].transport_time > Berth::min_transport_time + berth2berth_dis) {
				forward_flag = true;
				printf("ship %d %d\n", id, Berth::nearest_berth_id);
				transTime = Berth::min_transport_time + berth2berth_dis;
			} else {
				printf("go %d\n", id);
				transTime = berth[go_berth_id].transport_time;
			}
			
			int from = go_berth_id, to = -1;
			berth[go_berth_id].boat_num--;
			go_berth_id = -1;
			fprintf(stderr, "time not enough to sell, total load num: %d, total pull money: %d, current money: %d\n", total_load_num, total_pull_money, money);
			for (int i = 0; i < berth_num; i++) {
				LOG_see_berth(frame, berth[i].id, berth[i].boat_num, berth[i].load_num, berth[i].money);
			}
			LOG_see_boat(frame, id, taskT, from, to, c_capacity, status, transTime);
			return;
		}

		//// 倒数第二趟先去卖一趟 再回来收集
		//if (!timeout_flag && c_capacity > max_capacity * 0.8 && frame + berth[go_berth_id].transport_time + berth2berth_dis + 2 * Berth::max_transport_time >= 14998) {
		//	taskT = TO_SELL;
		//	printf("go %d\n", id);
		//	berth[go_berth_id].boat_num--;
		//	transTime = berth[go_berth_id].transport_time;
		//	go_berth_id = -1;
		//	LOG_INFO(frame, id, boat go v point);
		//	timeout_flag = true;
		//	return;
		//}

		if (c_capacity >= max_capacity) { // when arrived, go imedaitely, later remove and change to cpacity >= max_capacity * 0.9
			taskT = TO_SELL;
			berth[go_berth_id].current_boat_id = -1;
			if(berth[go_berth_id].transport_time > Berth::min_transport_time + berth2berth_dis) {
				forward_flag = true;
				printf("ship %d %d\n", id, Berth::nearest_berth_id);
				transTime = Berth::min_transport_time + berth2berth_dis;
				fprintf(stderr, "frame %d, boat %d, forward to sell\n", frame, id);
			} else {
				printf("go %d\n", id);
				transTime = berth[go_berth_id].transport_time;
			}
			int from = go_berth_id, to = -1;

			berth[go_berth_id].boat_num--;
			go_berth_id = -1;

			LOG_INFO(frame, id, boat go v point);
			fprintf(stderr, "full and to sell, total load num: %d, total pull money: %d, current money: %d\n", total_load_num, total_pull_money, money);
			for (int i = 0; i < berth_num; i++) {
				LOG_see_berth(frame, berth[i].id, berth[i].boat_num, berth[i].load_num, berth[i].money);
			}
			LOG_see_boat(frame, id, taskT, from, to, c_capacity, status, transTime);

		} 
		else if(berth[go_berth_id].load_num <= 0) { // if current berth have no goods
			float e_profit = float(INT_MIN);
			int select = -1;
			for(int i=0; i<berth_num; i++) {
				if(i == go_berth_id) continue;

				int estimate_gdn = berth[i].load_num;
				if(frame < 14999 - min(Berth::max_transport_time, Berth::min_transport_time + berth2berth_dis) * 2)
					estimate_gdn += int(berth[i].btotal_load_num * 1.0 * berth2berth_dis / frame);

				if(berth[i].current_boat_id != -1) estimate_gdn -= (max_capacity - boat[berth[i].current_boat_id].c_capacity);
				else if(berth[i].boat_num > 0) estimate_gdn -= max_capacity;
				if(estimate_gdn <= 0) continue;
				// if the profit of go to sell directly is larger than go to another berth load then to sell, do not go to another berth to laod
				if ((frame + min(berth[go_berth_id].transport_time, Berth::min_transport_time + berth2berth_dis) * 2 + berth2berth_dis < 14999)
					&& (float)c_money / min(berth[go_berth_id].transport_time, Berth::min_transport_time + berth2berth_dis) / 2 > ((float)c_money / c_capacity * min(max_capacity, c_capacity + estimate_gdn)) / (2 * min(berth[go_berth_id].transport_time, Berth::min_transport_time + berth2berth_dis) + berth2berth_dis)) continue;
								
				if (berth[i].boat_num < 2 && estimate_gdn + c_capacity <= max_capacity * 1.0 && // 这个1.0不同对不同的图影响也挺大
						frame + min(berth[i].transport_time, Berth::min_transport_time + berth2berth_dis) + berth2berth_dis <= 14999 - (max_capacity - c_capacity)/ berth[i].loading_speed) {
					float profit;
					if(frame < frame - 2 * min(berth[i].transport_time, Berth::min_transport_time + berth2berth_dis)) profit = estimate_gdn * 1.0 / berth[i].transport_time;
					else profit = estimate_gdn;
					if (e_profit < profit) {
						select = i;
						e_profit = profit;
					}
					// select = i;
					// break;
				}
			}
			if (select != -1) {
				berth[go_berth_id].current_boat_id = -1;
				berth[go_berth_id].boat_num--;
				int from = go_berth_id, to = select;
				go_berth_id = select;
				berth[select].boat_num++;
				printf("ship %d %d\n", id, select);
				transTime = 500;
				taskT = TO_LOAD;
				LOG_INFO(frame, id, boat go to another berth);
				LOG_see_boat(frame, id, taskT, from, to, c_capacity, status, transTime);
				return;
			}
			else if (frame < 15000 - 2 * min(Berth::max_transport_time, Berth::min_transport_time + berth2berth_dis)) {
				taskT = TO_SELL;
				berth[go_berth_id].current_boat_id = -1;
				if(berth[go_berth_id].transport_time > Berth::min_transport_time + berth2berth_dis) {
					forward_flag = true;
					printf("ship %d %d\n", id, Berth::nearest_berth_id);
					transTime = Berth::min_transport_time + berth2berth_dis;
					fprintf(stderr, "frame %d, boat %d, forward to sell\n", frame, id);
				} else {
					printf("go %d\n", id);
					transTime = berth[go_berth_id].transport_time;
				}
				int from = go_berth_id, to = -1;
				berth[go_berth_id].boat_num--;
				go_berth_id = -1;
				LOG_INFO(frame, id, boat go v point);
				fprintf(stderr, "to sell, total load num: %d, total pull money: %d, current money: %d\n", total_load_num, total_pull_money, money);
				for (int i = 0; i < berth_num; i++) {
					LOG_see_berth(frame, berth[i].id, berth[i].boat_num, berth[i].load_num, berth[i].money);
				}
				LOG_see_boat(frame, id, taskT, from, to, c_capacity, status, transTime);
			}
		}
		else {
			berth[go_berth_id].current_boat_id = id;
			auto [n, m] = berth[go_berth_id].load(c_capacity, max_capacity);
			c_capacity += n;
			c_money += m;
		}
	} 
	else if (taskT == TO_SELL) {
		if(forward_flag) {
			printf("go %d\n", id);
			forward_flag = false;
			fprintf(stderr, "frame %d, boat %d, forward sell done\n", frame, id);
		} else {
			LOG_INFO(frame, id, boat sell finished);
			c_capacity = 0;
			c_money = 0;
			go_berth_id = findAberthToLoad(berth, boat, frame);
			if (go_berth_id != -1) {
				if(berth[go_berth_id].transport_time > Berth::min_transport_time + berth2berth_dis) {
					forward_flag = true;
					printf("ship %d %d\n", id, Berth::nearest_berth_id);
					transTime = Berth::min_transport_time + berth2berth_dis;
					fprintf(stderr, "frame %d, boat %d, forward to load\n", frame, id);
				} else {
					printf("ship %d %d\n", id, go_berth_id);
					transTime = berth[go_berth_id].transport_time;
				}
				taskT = TO_LOAD;
				berth[go_berth_id].boat_num++;
				for (int i = 0; i < berth_num; i++) {
					LOG_see_berth(frame, berth[i].id, berth[i].boat_num, berth[i].load_num, berth[i].money);
				}
				LOG_see_boat(frame, id, taskT, -1, go_berth_id, c_capacity, status, transTime);
				fprintf(stderr, "sell done, total load num: %d, total pull money: %d, current money: %d\n", total_load_num, total_pull_money, money);
			} else {
				// printf("ship %d %d\n", id, Berth::nearest_berth_id);
				// transTime = Berth::min_transport_time;
				// taskT = TO_LOAD;
				// berth[Berth::nearest_berth_id].boat_num++;	
				// go_berth_id = Berth::nearest_berth_id;
				// LOG_see_boat(frame, id, taskT, -1, go_berth_id, c_capacity, status, transTime);
				transTime = 0;
			}
		}
	}
	else if (taskT == TO_LOAD) {
		if(forward_flag) {
			printf("ship %d %d\n", id, go_berth_id);
			forward_flag = false;
			fprintf(stderr, "frame %d, boat %d, forward to buy at mid trans\n", frame, id);

		} else
		{
			taskT = LOADING;
			goto LOAD;
		}
	}
}

int Boat::findAberthToLoad(vector<Berth>& berth, vector<Boat>& boat, int frame)
{
	float e_profit = float(INT_MIN);
	int target = -1;
	for (int b_i = 0; b_i < berth_num; b_i++) {
		// boat should go a berth which have greatest load num and no other boat at loading
		// can also estimate how many frame a boat still need to finish loading
		if(frame > 14999 - 2 * min(berth[b_i].transport_time, Berth::min_transport_time + berth2berth_dis)) continue;
		if(frame < 14999 - 2 * min(berth[b_i].transport_time, Berth::min_transport_time + berth2berth_dis) && berth[b_i].boat_num > 0) continue;
		int a = 0;
		int estimate_gdn = berth[b_i].load_num;
		if (frame < 15000 - 2 * Berth::max_transport_time)
			estimate_gdn += int(berth[b_i].btotal_load_num * 1.0 / frame * min(berth[b_i].transport_time, Berth::min_transport_time + berth2berth_dis));

		float profit = boat_loadNumAndTime_weight * (estimate_gdn - berth[b_i].boat_num * max_capacity) / berth[b_i].transport_time +
			1.0 * estimate_gdn; // temp only consider nearst berth and the num of boat go to that berth

		if (e_profit < profit) {
			e_profit = profit;
			target = b_i;
		}
	}
	return target;
}