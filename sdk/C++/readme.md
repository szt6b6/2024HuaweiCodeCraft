# next
1. goods map -> done
2. path conflig, collision, simplify path search, but also need to consider path conflic
   - now it can avoid path conflic in most situation, excep maps-3.10.txt
3. GET/PULL task queue -> simple version, done
4. lian tong field split(robot, berth, gds belong to which field, only in same field can make task), -> done
5. berth map size 4x4 problem, robot can not go into water -> done
6. gds map lock problem -> done
7. ship go to berth for load and goto virtual point for sell -> done, policy need to change, now is at game start, ship go to a berth
8. new path finding method, mind like water flow into ocean; pull operation do need to find path, get operation if in a berth, find a path is much faster; -> done
9. imporve distance estimate when init berth_dir_map, construct a variable berth_distance_map[][][], indicate the ditance from point to each berth -> done
10. boat policy designing -> improving
11. robot policy designing and improving -> improving

# Debug and see info
1. tips for debug, first open utils.h define 1 and can output stderr info, then redirect those err info into a file such as:
   - ./PreliminaryJudge -m maps/map1.txt ./build/main -l NONE 2> a.txt

# 参数
- 对有的地图 让空闲机器人不随机移动 分数会高一点 有的图则会低一点
- 船在决定直接回v点卖还是去其他港口装判断条件改一下 不同图表现也会不一样
- 机器人让路机制, 有的图规划路径尽量不产生冲分数会高一点 如map-3.10.txt