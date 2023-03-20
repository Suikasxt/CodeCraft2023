#include <iostream>
#include <stdio.h>
#include <cassert>
#include <ctime>
#include <cmath>
#include <cstring>
#include <algorithm>
#include "main.h"
#include "heap.h"
#include <stack>
#include "data.h"

vector<Studio> studio_list;
vector<Robot> robot_list;
vector<Studio*> studio_dict[10];
FILE* warning_output;
int money = 200000;
int frameID = 0;
char map[110][110];
int robot_target[4];
stack<int> robot_target_stack[4];
vector<int> robot_target_real[4];
vector<pair<int, int> > arr_list[4];
bool pre_work = false;
int map_num = 0;
void readUntilOK(){
    int nof_studio;
    scanf("%d %d\n", &money, &nof_studio);

    char line[1024];
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


    scanf("%s", line);
    assert(line[0] == 'O' && line[1] == 'K');
}

void readMap(){
    for (int i = 0; i < 10; i++){
        studio_dict[i].clear();
    }
    studio_list.clear();
    robot_list.clear();
    
    for (int i = 99; i >= 0; i--){
        scanf("%s", map[i]);
        for (int j = 0; j < 100; j++){
            if (map[i][j] == 'A'){
                int id = robot_list.size();
                robot_list.push_back(Robot(id, Point(j+0.5, i+0.5)*0.5));
            }
            else if (map[i][j] >= '1' && map[i][j] <= '9'){
                int id = studio_list.size();
                studio_list.push_back(Studio(id, map[i][j]-'0', Point(j+0.5, i+0.5)*0.5));
            }
        }
    }
    for (int i = 0; i < studio_list.size(); i++){
        studio_dict[studio_list[i].type].push_back(&(studio_list[i]));
    }
    char line[10];
    scanf("%s", line);
    assert(line[0] == 'O' && line[1] == 'K');
}

void stateOutput(){

#ifndef DEBUG_MODE
    return;
#endif

    char file_name[100];
    char str[1000];
    sprintf(file_name, "./debug/%d.txt", frameID);
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
void search(int width, int time){
    if (pre_work) return;
    Heap heap(width);
    int max_time = min(frameID + time, 9000);
    vector<UpdateRoad> road;
    Game* g = new Game(studio_list, robot_list, frameID, money);
    g->road_id = -1;
    g->passTime(0);
    heap.push(g);
    double max_value = -INF;
    int res_road_id = -1;
#ifdef DEBUG_MODE
    fprintf(warning_output, "\n\nSize %d %d\n", heap.size, res_road_id);
    fprintf(warning_output, "\n\nFrame: %d, money: %d, time: %d, value: %lf\n", g->frameID, g->money, g->nextTimeStep(), g->value);
    for (int i = 0; i < 4; i++){
        fprintf(warning_output, "ID: %d, Task: %d, Target: %d, Item: %d\n", i, g->robot_list[i].task_now, g->robot_list[i].target, g->robot_list[i].item);
    }
    fflush(warning_output);
#endif
    while (heap.size){
        Game* g = new Game(*heap.top());
        if (g->money > max_value && g->road_id != -1){
            max_value = g->money;
            res_road_id = g->road_id;
        }
        heap.pop(1, 0);
        g->passTime(g->nextTimeStep());
        if (g->frameID > max_time){
            continue;
        }
        double value_list[4][50];
        for (int i = 0; i < 4; i++){
            for (int j = 0; j < g->studio_list.size(); j++){
                value_list[i][j] = -INF;
            }
        }
        g->greedyWork(value_list);
        for (int i = 0; i < 4; i++){
            Robot* robot = &(g->robot_list[i]);
            if (robot->task_now != Task::NONE){
                continue;
            }
            double tmp[50];
            /*for (int k = 0; k < 4; k++){
                if (k == i) continue;
                if (robot_list[k].target != -1){
                    value_list[i][robot_list[k].target] -= 1000;
                }
                if (robot_list[k].last_target != -1){
                    value_list[i][robot_list[k].last_target] -= 1000;
                }
            }*/
            memcpy(tmp, value_list[i], sizeof(tmp));
            sort(tmp, tmp+studio_list.size());
            for (int j = 0; j < g->studio_list.size(); j++){
                //每次选贪心策略分数最高的K个来扩展，也是计算的时候可以调的参数之一
                int search_width_K = 2;
                if (value_list[i][j] <= tmp[g->studio_list.size() - 1 - search_width_K] || value_list[i][j] <= -INF+EPS){
                    continue;
                } 
                Studio* studio = &(g->studio_list[j]);
                Game* new_g = new Game(*g);
                if (robot->item){
                    new_g->robot_list[i].task_now = Task::SELL;
                }else{
                    new_g->robot_list[i].task_now = Task::BUY;
                }
                new_g->robot_list[i].target = j;
                if (heap.push(new_g)){
                    new_g->road_id = road.size();
                    road.push_back(UpdateRoad(g->road_id, i, j));
                }
            }
            if (robot->item == 0){
                Game* new_g = new Game(*g);
                new_g->robot_list[i].task_now = Task::STOP;
                new_g->robot_list[i].target = -1;
                if (heap.push(new_g)){
                    new_g->road_id = road.size();
                    road.push_back(UpdateRoad(g->road_id, i, -1));
                }
            }
            break;
        }
    }
    memset(robot_target, -1, sizeof(robot_target));
    for (int i = 0; i < 4; i++){
        while (!robot_target_stack[i].empty()) robot_target_stack[i].pop();
    }
    while (res_road_id!=-1){
        robot_target[road[res_road_id].robot_id] = road[res_road_id].target_id;
        robot_target_stack[road[res_road_id].robot_id].push(road[res_road_id].target_id);
        res_road_id = road[res_road_id].pre_id;
    }
    
    fprintf(stderr, "{%d, %d, %d}\n", studio_list.size(), studio_list[0].type, studio_list[1].type);
    for (int i = 0; i < 4; i++){
        stack<int> tmp(robot_target_stack[i]);
        fprintf(stderr, "{");
        while (!tmp.empty()){
            fprintf(stderr, "%d, ", tmp.top());
            tmp.pop();
        } 
        fprintf(stderr, "-1},\n");
    }
    fprintf(stderr, "Except money: %lf\n", max_value);
}

int last_design = -INF;
void work(){
    //本地计算的的时候主要关注下面每个多长时间重新计算，以及每次计算用多大的beam，上面search的部分扩展的宽度也是可以调的
    int interval = 1000;
    int beam_width = 1000;
    bool redesign = false;
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->task_now == Task::WAIT){
            robot->task_now = Task::NONE;
        }
        money += robot->update(studio_list, frameID, true);
        if (robot->task_now == Task::NONE && frameID > last_design + interval){
            redesign = true;
        }
    }
    if (redesign){
        search(beam_width, 9000);
        last_design = frameID;
    }

    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->task_now != Task::NONE){
            robot->dispatch(&(studio_list[robot->target]), true);
        }else if (robot_target_stack[robot->id].empty() || robot_target_stack[robot->id].top() == -1){
            robot->task_now = Task::WAIT;
        }else{
            if (robot_target_stack[robot->id].empty()){
                continue;
            }
            robot_target[robot->id] = robot_target_stack[robot->id].top();
            robot_target_real[robot->id].push_back(robot_target[robot->id]);
            robot_target_stack[robot->id].pop();
            robot->dispatch(&(studio_list[robot_target[robot->id]]), true);
            fprintf(stderr, "%d dispatch %d %d Money: %d\n", frameID, robot->id, robot->target, money);
        }
    }
    #ifdef DEBUG_MODE
        for (int i = 0; i < 4; i++){
            fprintf(warning_output, "ID: %d, Target: %d\n", i, robot_list[i].target);
        }
        fflush(warning_output);
    #endif
    
    /*
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        money += robot->update(studio_list, frameID, true);
    }
    if (frameID == 1){
        for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
            robot->dispatch(&studio_list[robot_target[robot->id]]);
        }
    }
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        //robot->task_now = Task::NONE;
        if (robot->task_now == Task::NONE){
            Game g(studio_list, robot_list, frameID, money);
            int target_id = g.MCTS(robot->id);
            if (target_id == -1){
                continue;
            }
            #ifdef DEBUG_MODE
                fprintf(warning_output, "Frame: %d, ID: %d, Target: %d\n", frameID, robot->id, target_id);
                fflush(warning_output);
            #endif
            robot->dispatch(&studio_list[target_id], true);
            //fprintf(stderr, "%d dispatch %d %d\n", frameID, robot->id, robot->target);
        }else if (robot->target != -1){
            robot->dispatch(&studio_list[robot->target], true);
        }
    }
    */
    


    for (auto robot_A = robot_list.begin(); robot_A != robot_list.end(); robot_A++){
        for (auto robot_B = robot_A + 1; robot_B != robot_list.end(); robot_B++){
            Point last_delta = (robot_A->position - robot_B->position);
            for (double time = 0.01; time < 1; time += 0.02){
                Point next_delta = (robot_A->position - robot_B->position) + (robot_A->velocity - robot_B->velocity) * time;
                if (abs(next_delta) >= abs(last_delta) - EPS){
                    break;
                }
                last_delta = next_delta;
                if (abs(next_delta) > (robot_A->getRadius() + robot_B->getRadius())){
                    continue;
                }
                Point center = (robot_A->position + robot_B->position + (robot_A->velocity + robot_B->velocity) * time) * 0.5;
                Point delta_A = center - robot_A->position;
                Point delta_B = center - robot_B->position;
                double angle_A = atan2(delta_A.y, delta_A.x);
                double angle_B = atan2(delta_B.y, delta_B.x);
                int flag_A = angleAdjust(angle_A - robot_A->angle) > 0? 1: -1;
                int flag_B = angleAdjust(angle_B - robot_B->angle) > 0? 1: -1;
                robot_A->setAngleV(robot_A->angle_v - flag_A*3);
                robot_B->setAngleV(robot_B->angle_v - flag_B*3);
                if (time < 0.1){
                    robot_A->setAngleV(robot_A->angle_v - flag_A*2);
                    robot_B->setAngleV(robot_B->angle_v - flag_B*2);
                }
                
                Point target_delta_A = robot_A->target==-1?Point(INF, INF) : (robot_A->position - studio_list[robot_A->target].position);
                Point target_delta_B = robot_B->target==-1?Point(INF, INF) : (robot_B->position - studio_list[robot_B->target].position);
                if (robot_A->target == robot_B->target){
                    if (abs(target_delta_A) > abs(target_delta_B)){
                        robot_A->setVelocity(abs(robot_A->velocity)-3);
                    }else{
                        robot_B->setVelocity(abs(robot_B->velocity)-3);
                    }
                }
#ifdef DEBUG_MODE
                fprintf(warning_output, "Collision %d %d time: %lf center: %lf %lf angle: %lf %lf\n", robot_A->id, robot_B->id, time, center.x, center.y, angle_A, angle_B);
                fprintf(warning_output, "%lf %lf\n", abs(next_delta), abs(last_delta));
#endif
                break;
            }
        }
    }
}

int main() {

    readMap();
    stateOutput();
    for (int i = 0; i < PRE_WORK_MAP_NUM; i++){
        if (studio_list.size() == MAP_FEATURE[i][0] && studio_list[0].type == MAP_FEATURE[i][1] && studio_list[1].type == MAP_FEATURE[i][2]){
            map_num = i+1;
            //这个开关用来切换计算模式还是推理模式，break开了就是本地计算，最后会打一个结果到warning.txt，从里面把数据贴到data.h就可以
            break;
            pre_work = true;
            for (int j = 0; j < 4; j++){
                while (!robot_target_stack[j].empty()) robot_target_stack[j].pop();
                int k = WORK_LIST_LENGTH - 1;
                while (PRE_WORK_DATA[i][j][k] != -1) k--;
                fprintf(stderr, "%d %d %d\n", k, j, PRE_WORK_DATA[i][j][k]);
                while (k>=0){
                    robot_target_stack[j].push(PRE_WORK_DATA[i][j][k]);
                    k--;
                }
            }
            break;
        }
    }
#ifdef DEBUG_MODE
    char file_name[100];
    sprintf(file_name, "warning_%d.txt", map_num);
    warning_output = fopen(file_name, "w");
#endif
    puts("OK");
    fflush(stdout);
    while (scanf("%d", &frameID) != EOF) {
        //fprintf(stderr, "%d\n", frameID);
        readUntilOK();
        printf("%d\n", frameID);

        work();
#ifdef DEBUG_MODE
        fprintf(warning_output, "End frame ID: %d, money: %d, clock: %.6lfs %lf %lf\n", frameID, money, double(clock())/CLOCKS_PER_SEC, robot_list[0].position.x, robot_list[0].position.y);
#endif
        printf("OK\n", frameID);

        stateOutput();
        fflush(stdout);
    }
#ifdef DEBUG_MODE
    for (int i = 0; i < 4; i++){
        double angle = 0;
        for (int j = 1; j < arr_list[i].size(); j++){
            Point delta = studio_list[arr_list[i][j].first].position - studio_list[arr_list[i][j-1].first].position;
            double angle_now = atan2(delta.y, delta.x);
            Robot robot(0, studio_list[arr_list[i][j-1].first].position);
            robot.angle = angle;
            robot.target = arr_list[i][j].first;
            fprintf(warning_output, "%lf %lf %d %d\n",
            abs(delta), angleAdjust(angle_now - angle), arr_list[i][j].second - arr_list[i][j-1].second, moveTimePredict(&robot));
            angle = angle_now;
        }
    }
    fprintf(warning_output, "\n\n\n\n");
    fflush(warning_output);
    for (int i = 0; i < 4; i++){
        stack<int> tmp(robot_target_stack[i]);
        fprintf(warning_output, "{");
        for (auto j = robot_target_real[i].begin(); j != robot_target_real[i].end(); j++){
            fprintf(warning_output, "%d, ", *j);
        } 
        fprintf(warning_output, "-1},\n");
    }
    fprintf(warning_output, "\n\n\n\n");
    fflush(warning_output);
#endif
#ifdef DEBUG_MODE
    fflush(warning_output);
#endif
    
#ifdef DEBUG_MODE
    fclose(warning_output);
#endif
    return 0;
}
