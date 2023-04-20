#include <iostream>
#include <stdio.h>
#include <cassert>
#include <ctime>
#include <math.h>
#include <cstring>
#include <algorithm>
#include "main.h"
#include <stack>
#include "data.h"
#include <string.h>
#include <stdlib.h>
#include "map.h"
#include <queue>
#include <pthread.h>
#include <atomic>
#include <unistd.h>

vector<Studio> studio_list;
vector<Studio> enemy_studio_list;
vector<Robot> robot_list;
FILE* warning_output;
int money = 200000;
int frameID = 0;
int last_avoid[4];
char map[110][110];
int robot_target[4];
stack<pair<int, int> > robot_target_stack[4];
vector<pair<int, int>> robot_target_real[4];
vector<pair<int, int> > arr_list[4];
bool pre_work = false;
bool greedy_work = true;
int map_num = 0;


bool new_block_main[50*MAP_SCALE][50*MAP_SCALE];
double map_dist_main[2][50][50*MAP_SCALE][50*MAP_SCALE];
pair<int, int> map_target_main[2][50][50*MAP_SCALE][50*MAP_SCALE];
bool walk_available_main[2][50*MAP_SCALE][50*MAP_SCALE];
double studio_dist_main[50][50];
Point studio_dist_addition_main[50][50];


char team[10];
int time_count = 0;
int total_time_count = 0;

pthread_t search_thread;
std::atomic<bool> roadSearching(false);


void* searchThread(void* args){
    int time_to_sleep = 10; //ms
    while(frameID < 12000){
        while(roadSearching.load() == false){
            usleep(time_to_sleep*1000); //sleep 10ms
        }
        mapInit();
        roadSearching.store(false);
    }
}
void readUntilOK(){
    int nof_studio;
    scanf("%d %d\n", &money, &nof_studio);

    char line[40960];
    for (int i = 0; i < nof_studio; i++){
        fgets(line, sizeof(line), stdin);
        studio_list[i].readFromString(line);
    }
    
    for (int i = 0; i < robot_list.size(); i++){
        fgets(line, sizeof(line), stdin);
        robot_list[i].readFromString(line);
    }
    for (int i = 0; i < 4; i++){
        if (robot_list[i].studio_id == -1){
            continue;
        }
        if (arr_list[i].size() && robot_list[i].studio_id == arr_list[i][arr_list[i].size()-1].first){
            continue;
        }
        arr_list[i].push_back(make_pair(robot_list[i].studio_id, frameID));
    }

    vector<pair<int, int>> new_block_list;
    for (int x = 0; x < 50*MAP_SCALE; x++)
    for (int y = 0; y < 50*MAP_SCALE; y++){
        if (new_block_main[x][y]){
            new_block_list.push_back(make_pair(x, y));
        }
    }

    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 360; j++){
            scanf("%lf", robot_list[i].radar+j);
        }
        for (int l = 0; l < new_block_list.size(); l++){
            if (new_block_list[l].first < 0){
                continue;
            }
            Point block_pos = Discrete2Continuous(new_block_list[l]);
            Point delta = block_pos - robot_list[i].position;
            double angle = atan2(delta.y, delta.x);
            double delta_angle = angleAdjust(angle - robot_list[i].angle);
            if (delta_angle < 0){
                delta_angle +=2*M_PI;
            }
            int j = round(delta_angle/M_PI*180);
            if (robot_list[i].radar[j] > abs(delta) - 0.05){
                new_block_main[new_block_list[l].first][new_block_list[l].second] = false;
                new_block_list[l] = make_pair(-1, -1);
            }
        }
        for (int j = 0; j < 360; j++){
            double angle = (M_PI*j/180) + robot_list[i].angle;
            Point block_pos = Point(cos(angle), sin(angle)) * robot_list[i].radar[j] + robot_list[i].position;
            pair<int, int> coord = Continuous2Discrete(block_pos);
            bool is_my_robot = false;
            for (int k = 0; k < 4 && !is_my_robot; k++){
                if (abs(block_pos - robot_list[k].position) < robot_list[k].getRadius() + EPS + 0.01){
                    is_my_robot = true;
                }
            }
            if (is_my_robot){
                continue;
            }
            for (int x = coord.first; x < coord.first + 2; x++)
            for (int y = coord.second; y < coord.second + 2; y++){
                if (isBlock(x, y)){
                    continue;
                }
                new_block_main[x][y] = true;
            }
        }
    }
    //studio_list = enemy_studio_list;
    scanf("%s", line);
    assert(line[0] == 'O' && line[1] == 'K');
}


void stateOutput(){

#ifndef DEBUG_MODE
    return;
#endif

    char file_name[100];
    char str[1000];
    sprintf(file_name, "./debug/%d_%s.txt", frameID, team);
    FILE* file = fopen(file_name, "w");

    fprintf(file, "**********output studio\n");
    //output studio
    for (int i = 0; i < studio_list.size(); i++){
        studio_list[i].outputToString(str);
        fprintf(file, "%s\n", str);
    }

    //output robot
    fprintf(file, "**********output robot\n");
    for (int i = 0; i < robot_list.size(); i++){
        robot_list[i].outputToString(str);
        fprintf(file, "%s\n", str);
    }
    fclose(file);
}
void OutputPath(vector<Point> path[4]){
#ifdef DEBUG_MODE
    for (int i = 0; i < 4; i++){
        fprintf(warning_output, "    path[%d]=[", i);
        for (int j = 0; j < path[i].size(); j++){
            Point p = path[i][j];
            fprintf(warning_output, "(%lf, %lf), ", p.x, p.y);
        }
        fprintf(warning_output, "]\n");
    }
#endif
}


void greedyWork(){
    if (roadSearching.load() == false){
        memcpy(map_dist_main, map_dist, sizeof(map_dist));
        memcpy(map_target_main, map_target, sizeof(map_target));
        memcpy(walk_available_main, walk_available, sizeof(walk_available));
        memcpy(studio_dist_main, studio_dist, sizeof(studio_dist));
        memcpy(studio_dist_addition_main, studio_dist_addition, sizeof(studio_dist_addition));
        

        memcpy(new_block, new_block_main, sizeof(new_block));
#ifdef DEBUG_MODE
    for (int i = 50*MAP_SCALE-1; i >= 0; i--){
        for (int j = 0; j < 50*MAP_SCALE; j++){
            fprintf(warning_output, "%d", isBlock(j, i));
        }
        fprintf(warning_output, "\n");
        fflush(warning_output);
    }
#endif
        roadSearching.store(true);
    }
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        money += robot->update(studio_list, frameID, true);
        //robot->task_now = Task::NONE;
    }
    Game g(studio_list, robot_list, frameID, money);
    g.greedyWork();
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (g.robot_list[robot->id].task_now == Task::BUY || g.robot_list[robot->id].task_now == Task::SELL){
            robot->dispatch(&(studio_list[g.robot_list[robot->id].target]), -1, true);
            robot->additional_target_position = g.robot_list[robot->id].additional_target_position;
            robot->next_target = g.robot_list[robot->id].next_target;
        }
    }
    
    //CollisionAvoid
    //先把初始路径都找出来
    vector<Point> path[4];
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->target != -1){
            pair<int, int> coord = Continuous2DiscreteRound(robot->position);
            pair<int, int> target = map_target_main[robot->item>0][robot->target][coord.first][coord.second];
            path[robot->id].push_back(Discrete2Continuous(target));
            while(target != map_target_main[robot->item>0][robot->target][target.first][target.second]){
                target = map_target_main[robot->item>0][robot->target][target.first][target.second];
                path[robot->id].push_back(Discrete2Continuous(target));
            }
            if (robot->item == 0){
                path[robot->id][path[robot->id].size()-1] = robot->additional_target_position;
            }
        }
    }
    const int SIM_TIME = 50;
    double r[4];
    
    for (int i = 0; i < 4; i++)
    if (path[i].size()){
        r[i] = robot_list[i].getRadius();
        Robot* robot = &(robot_list[i]);
        int l = 0;
        for (int j = 20; j>=0; j--){
            if (l + (1<<j) < path[i].size() && DirectWalk(r[i], robot->position, path[i][l+(1<<j)])){
                l += 1<<j;
            }
        }
        Point target_position = path[i][l];
        robot_list[i].goToTargetPosition(target_position, true);
        robot_list[i].target_angle_future = robot_list[i].angle_v;
    }

    

    //核心的回避算法
    OutputPath(path);
    int avoid[4] = {};
    for (int t = 0; t < 3; t++){
        for (int i = 0; i < 4; i++){
            if (path[i].size())
            for (int j = 0; j < SIM_TIME && path[i].size() < SIM_TIME; j++){
                path[i].push_back(path[i][path[i].size()-1]);
            }
        }
        double extra_cost[4];
        vector<Point> new_path[4];
        int road_occu[50*MAP_SCALE][50*MAP_SCALE];

        for (int i = 0; i < 4; i++){
            extra_cost[i] = INF+EPS;
        }

        int frame[4] = {};
        Point postion[4];
        for (int i = 0; i < 4; i++){
            postion[i] = robot_list[i].position;
        }
        bool moved = true;
        bool finish = false;
        //检查一下有没有碰撞发生
        while(moved && !finish){
            moved = false;
            finish = true;
            for (int i = 0; i < 4; i++)
            if (frame[i] < min(int(path[i].size()), SIM_TIME)){
                bool collision = false;
                for (int j = 0; j < 4; j++)
                if (i!=j && (abs2(path[i][frame[i]] - postion[j]) < sqr(r[i] + r[j]))
                    && abs2(path[i][frame[i]] - postion[j]) < abs2(postion[i] - postion[j])){
                    collision = true;
                }
                if (!collision){
                    postion[i] = path[i][frame[i]];
                    frame[i]++;
                    moved = true;
                }
                finish &= frame[i] == min(int(path[i].size()), SIM_TIME);
            }
        }
        if (finish){
            break;
        }

        //对每个机器人分别尝试回避，计算代价
        for (int i = 0; i < 4; i++)
        if (frame[i] != min(int(path[i].size()), SIM_TIME)){
            for (int x = 0; x < 50*MAP_SCALE; x++)
            for (int y = 0; y < 50*MAP_SCALE; y++){
                road_occu[x][y] = SIM_TIME;
            }
            for (int j = 0; j < 4; j++)
            if (j!=i){
                for (int k = 0; k < path[j].size() && k < SIM_TIME; k++){
                    pair<int, int> coord = Continuous2DiscreteRound(path[j][k]);
                    for (int dx = -6; dx < 6; dx++)
                    for (int dy = -6; dy < 6; dy++){
                        int x = coord.first + dx;
                        int y = coord.second + dy;
                        if (!isBlock(x, y) && abs2(Discrete2Continuous(make_pair(x, y)) - path[j][k]) < sqr(r[i] + r[j] + 0.15)){
                            road_occu[x][y] = min(road_occu[x][y], k);
                        }
                    }
                }
            }
            //搜索一条回避路径
            queue<pair<int, pair<int, int>>> q;
            pair<int, int> target(-1, -1);
            pair<int, int> pre[50*MAP_SCALE][50*MAP_SCALE];
            memset(pre, -1, sizeof(pre));
            
            pair<int, int> coord = Continuous2DiscreteRound(robot_list[i].position);
            pre[coord.first][coord.second] = coord;
            q.push(make_pair(0, coord));
            while (!q.empty()){
                int f = q.front().first;
                pair<int, int> pos = q.front().second;
                q.pop();
#ifdef DEBUG_MODE
                //fprintf(warning_output, "%d %d %d %d\n", f, pos.first, pos.second, road_occu[pos.first][pos.second]);
#endif
                double cost = -INF;
                if (robot_list[i].target != -1){
                    cost = map_dist_main[robot_list[i].item>0][robot_list[i].target][pos.first][pos.second] - map_dist_main[robot_list[i].item>0][robot_list[i].target][coord.first][coord.second];
                }
                cost += f*0.1;
                cost += (SIM_TIME - road_occu[pos.first][pos.second]) * 1000;
                if (cost < extra_cost[i]){
                    target = pos;
                    extra_cost[i] = cost;
                    //continue;
                }
                if (f > SIM_TIME){
                    continue;
                }
                for (int d = 0; d < 4; d++){
                    bool collision_now = false;
                    int x = pos.first + DIRECTION[d][0];
                    int y = pos.second + DIRECTION[d][1];
                    pair<int, int> next = make_pair(x, y);
                    if (isBlock(x, y) || !walk_available_main[robot_list[i].item>0][x][y]){
                        continue;
                    }
                    if (pre[x][y].first != -1){
                        continue;
                    }
                    for (int j = 0; j < 4; j++)
                    if (j!=i){
                        if (path[j].size() > f && abs2(Discrete2Continuous(next) - path[j][f]) < sqr(r[i] + r[j] + 0.05) &&
                            abs2(Discrete2Continuous(next) - path[j][f]) - EPS <= abs2(Discrete2Continuous(pos) - path[j][f])){
                            collision_now = true;
                        }
                    }
                    if (collision_now){
                        continue;
                    }
                    if (road_occu[x][y] < road_occu[pos.first][pos.second]){
                        continue;
                    }
                    pre[x][y] = pos;
                    q.push(make_pair(f+1, next));
                }
            }
            if (target.first != -1){
                while (target != pre[target.first][target.second]){
                    new_path[i].push_back(Discrete2Continuous(target));
                    target = pre[target.first][target.second];
                }
                new_path[i].push_back(Discrete2Continuous(target));
                reverse(new_path[i].begin(), new_path[i].end());
            }
        }
#ifdef DEBUG_MODE
        fprintf(warning_output, "%lf %lf %lf %lf\n", extra_cost[0], extra_cost[1], extra_cost[2], extra_cost[3]);
#endif
        //找代价最小的一个实际执行
        for (int i = 0; i < 4; i++)
        if (last_avoid[i]){
            extra_cost[i] -= 5 * last_avoid[i];
        }
        int min_cost_index = 0;
        for (int i = 0; i < 4; i++)
        if (extra_cost[i] < extra_cost[min_cost_index]){
            min_cost_index = i;
        }
        if (extra_cost[min_cost_index] >= INF){
            break;
        }
        path[min_cost_index] = new_path[min_cost_index];
        avoid[min_cost_index] = 5 - t;
        OutputPath(path);
    }
    for (int i = 0; i < 4; i++){
        last_avoid[i] = avoid[i];
    }

    //为了避免路径横平竖直，找到最远的一个可以直接过去的点
    for (int i = 0; i < 4; i++)
    if (path[i].size()){
        Robot* robot = &(robot_list[i]);
        int l = 0;
        for (int j = 20; j>=0; j--){
            if (l + (1<<j) < path[i].size() && DirectWalk(r[i], robot->position, path[i][l+(1<<j)])){
                l += 1<<j;
            }
        }
        for (int j = 0; j < 4; j++)
        if (j!=i){
            while (l && point2line_sqr(robot_list[j].position, robot->position, path[i][l]) < sqr(r[i]+r[j])){
                l--;
            }
        }
        Point target_position = path[i][l];
        robot_list[i].goToTargetPosition(target_position, true);
        if (abs(robot_list[i].position - target_position) < 0.01){
            //fprintf(stderr, "%d %lf\n", i, robot_list[i].target_angle_future);
            robot_list[i].setAngleV(robot_list[i].target_angle_future, true);
        }
    }else{
        robot_list[i].setAngleV(robot_list[i].target_angle_future);
    }

    //加这个是实现等别的机器人回避的时候不要撞上去
    for (int i = 0; i < 4; i++)
    if (robot_list[i].position_v > 0 && !avoid[i])
    for (int j = 0; j < 4; j++)
    if (i!=j){
        Point next_pos = robot_list[i].position + Point(cos(robot_list[i].angle), sin(robot_list[i].angle))*0.2;
        double dist_now = abs(robot_list[i].position - robot_list[j].position);
        double dist_next = abs(next_pos - robot_list[j].position);
        if (dist_next < dist_now && dist_next < r[i]+r[j]-0.1){
            robot_list[i].setVelocity(-2, true);
        }
    }
    /*for (int i = 0; i < 4; i++)
    if (robot_list[i].position_v > 2){
        Point next_pos = robot_list[i].position + Point(cos(robot_list[i].angle), sin(robot_list[i].angle))*0.3;
        pair<int, int> coord = Continuous2DiscreteRound(next_pos);
        
        for (int dx = -6; dx < 6; dx++)
        for (int dy = -6; dy < 6; dy++){
            int x = coord.first + dx;
            int y = coord.second + dy;
            if (isBlock(x, y) && abs(Discrete2Continuous(make_pair(x, y)) - next_pos) < r[i]){
                robot_list[i].setVelocity(2, true);
            }
        }
    }*/
    //DWACollisionAvoid();
}

int main(int argc, char *argv[]) {
    scanf("%s", team);
    is_red = team[0]=='R';

#ifdef DEBUG_MODE
    char file_name[100];
    sprintf(file_name, "warning_%d_%s.txt", map_num, team);
    warning_output = fopen(file_name, "w");
#endif
    readMap();
    pthread_create(&search_thread, 0, searchThread, 0);
    stateOutput();
    puts("OK");
    fflush(stdout);
    while (scanf("%d", &frameID) != EOF) {
        //fprintf(stderr, "%d\n", frameID);
        readUntilOK();
        printf("%d\n", frameID);

        if (!pre_work){
            greedyWork();
        }
#ifdef DEBUG_MODE
        fprintf(warning_output, "End frame ID: %d, money: %d, clock: %.6lfs %lf %lf\n", frameID, money, double(clock())/CLOCKS_PER_SEC, robot_list[0].position.x, robot_list[0].position.y);
        fflush(warning_output);
#endif
        printf("OK\n", frameID);

        stateOutput();
        fflush(stdout);
    }
#ifdef DEBUG_MODE
    fflush(warning_output);
#endif
    
#ifdef DEBUG_MODE
    fclose(warning_output);
#endif
    return 0;
}
