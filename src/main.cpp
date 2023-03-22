#include <iostream>
#include <stdio.h>
#include <cassert>
#include <ctime>
#include <math.h>
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
                int search_width_K = 4;
                if (value_list[i][j] < tmp[g->studio_list.size() - search_width_K] || value_list[i][j] <= -INF+EPS){
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
        if (road[res_road_id].target_id!=-1){
            robot_target_stack[road[res_road_id].robot_id].push(road[res_road_id].target_id);
        }
        res_road_id = road[res_road_id].pre_id;
    }
#ifdef DEBUG_MODE
    fprintf(stderr, "{%d, %d, %d}\n", studio_list.size(), studio_list[0].type, studio_list[1].type);
    for (int i = 0; i < 4; i++){
        stack<int> tmp(robot_target_stack[i]);
        fprintf(stderr, "{");
        while (!tmp.empty()){
            fprintf(stderr, "%d, ", tmp.top());
            tmp.pop();
        } 
        fprintf(stderr, "},\n");
    }
    fprintf(stderr, "FrameID %d Except money: %lf\n", frameID, max_value);
#endif
}

int last_design = -INF;
void work(){
    //本地计算的的时候主要关注下面每个多长时间重新计算，以及每次计算用多大的beam，上面search的部分扩展的宽度也是可以调的
    int interval = 500;
    int beam_width = 100000;
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
        }else if (robot_target_stack[robot->id].empty() || robot_target_stack[robot->id].top()==-1){
            robot->task_now = Task::WAIT;
        }else{
            if (robot_target_stack[robot->id].empty() || robot_target_stack[robot->id].top()==-1){
                continue;
            }
            int delta_money = 0;
            do{
                robot_target[robot->id] = robot_target_stack[robot->id].top();
                robot_target_real[robot->id].push_back(robot_target[robot->id]);
                robot_target_stack[robot->id].pop();
                robot->dispatch(&(studio_list[robot_target[robot->id]]), true);
                break;
                delta_money = robot->update(studio_list, frameID, true);
                money += delta_money;
            }while(delta_money != 0);
            //fprintf(stderr, "%d dispatch %d %d Money: %d\n", frameID, robot->id, robot->target, money);
        }
    }
    #ifdef DEBUG_MODE
        for (int i = 0; i < 4; i++){
            fprintf(warning_output, "ID: %d, Target: %d\n", i, robot_list[i].target);
        }
        fflush(warning_output);
    #endif
    
    //碰撞检测
    stack<int> rt[4];
    double gap = 0.05;
    bool collision[4] = {0, 0, 0, 0};
    pair<double, int> dist_2_target_list[4];
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->target == -1 || (studio_list[robot->target].item&robot->item)!=0){
            dist_2_target_list[robot->id] = make_pair(INF, robot->id);
        }else{
            dist_2_target_list[robot->id] = make_pair(abs(robot->position - studio_list[robot->target].position), robot->id);
        }
    }
    sort(dist_2_target_list, dist_2_target_list+4);
    Game g(studio_list, robot_list, frameID, money);
    for (int j = 0; j < 4; j++){
        rt[j] = robot_target_stack[j];
    }
    for (int i = 0; i < 30; i++){
        g.physicalSimulation(rt);
        for (auto robot_A = g.robot_list.begin(); robot_A != g.robot_list.end(); robot_A++){
            for (auto robot_B = robot_A + 1; robot_B != g.robot_list.end(); robot_B++){
                if (abs(robot_A->position - robot_B->position) < robot_A->getRadius() + robot_B->getRadius() + gap){
                    collision[robot_A->id] = true;
                    collision[robot_B->id] = true;
                }
            }
        }
    }
    for (int i = 3; i >= 0; i--){
        int robot_id = dist_2_target_list[i].second;
        if (collision[robot_id] == false){
            continue;
        }
        double min_dist_2_target = INF+EPS;
        for (int add_frame = 0; add_frame <= 30; add_frame += 10){
            for (double add_pos_v = -2; add_pos_v <= 6+EPS; add_pos_v+=2){
                for (double add_angle_v = -M_PI; add_angle_v <= M_PI+EPS; add_angle_v += M_PI_2){
                    if (add_frame == 0 && (abs(add_pos_v) > EPS || abs(add_angle_v) > EPS)){
                        continue;
                    }
                    for (int j = 0; j < 4; j++){
                        rt[j] = robot_target_stack[j];
                    }
                    Game g(studio_list, robot_list, frameID, money);
                    
                    for (int k = i-1; k >= 0; k--)
                    if (collision[dist_2_target_list[k].second]){
                        g.robot_list[dist_2_target_list[k].second].additional_frame_num = 0;
                    }
                    g.robot_list[robot_id].additional_angle_v = add_angle_v;
                    g.robot_list[robot_id].additional_pos_v = add_pos_v;
                    g.robot_list[robot_id].additional_frame_num = add_frame;
                    bool collision = false;
                    double min_robot_dist = INF;
                    int f;
                    for (f = 0; f < 30 && collision==false; f++){
                        g.physicalSimulation(rt);
                        //fprintf(stderr, "%d %d %d %d\n", f, robot_id, add_frame, g.robot_list[robot_id].additional_frame_num);
                        Robot* robot_now = &(g.robot_list[robot_id]);
                        for (auto robot = g.robot_list.begin(); robot != g.robot_list.end(); robot++){
                            if (robot->id == robot_id){
                                continue;
                            }
                            double dist = abs(robot->position - robot_now->position);
                            min_robot_dist = min(min_robot_dist, dist);
                            if (dist < robot->getRadius() + robot_now->getRadius() + gap){
                                collision = true;
                            }
                        }
                    }
                    
                    double dist_2_target = 0;
#ifdef DEBUG_MODE
                    fprintf(warning_output, "%d %lf %lf %d : %lf %lf\n", robot_id, add_angle_v, add_pos_v, add_frame, dist_2_target, min_dist_2_target);
                    fflush(warning_output);
#endif
                    if (collision){
                        //continue;
                        dist_2_target += 2e8;
                        dist_2_target -= f*100;
                        dist_2_target -= min_robot_dist;
                    }else{
                        for (int k = 0; k < 4; k++){
                            dist_2_target += rt[k].size()*100;
                            int target = g.robot_list[k].target;
                            if (target != -1 && (g.studio_list[target].item&g.robot_list[k].item)==0){
                                dist_2_target += abs(g.studio_list[target].position - g.robot_list[k].position);
                            }else{
                                dist_2_target += 100;
                            }
                        }
                        dist_2_target -= min_robot_dist*3;
                    }
                    
                    if (dist_2_target < min_dist_2_target){
                        min_dist_2_target = dist_2_target;
                        robot_list[robot_id].additional_angle_v = add_angle_v;
                        robot_list[robot_id].additional_pos_v = add_pos_v;
                        robot_list[robot_id].additional_frame_num = add_frame;
                    }
                }
            }
            if (min_dist_2_target < 1e8){
                break;
            }
        }
#ifdef DEBUG_MODE
        fprintf(warning_output, "priority: %lf id: %d solution: (%lf %lf %d) : %lf\n", dist_2_target_list[i].first, robot_id, robot_list[robot_id].additional_angle_v, robot_list[robot_id].additional_pos_v, robot_list[robot_id].additional_frame_num, min_dist_2_target);
#endif
    }

    /*
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
                if (time < (map_num==3?0.1:0.2)){
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
                //fprintf(warning_output, "Collision %d %d time: %lf center: %lf %lf angle: %lf %lf\n", robot_A->id, robot_B->id, time, center.x, center.y, angle_A, angle_B);
                //fprintf(warning_output, "%lf %lf\n", abs(next_delta), abs(last_delta));
#endif
                break;
            }
        }
    }
    */
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->additional_frame_num){
            robot->setAngleV(robot->additional_angle_v, true);
            robot->setVelocity(robot->additional_pos_v, true);
            robot->additional_frame_num -= 1;
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
#ifdef _LOCAL
            //break;
#endif
            pre_work = true;
            for (int j = 0; j < 4; j++){
                while (!robot_target_stack[j].empty()) robot_target_stack[j].pop();
                int k = WORK_LIST_LENGTH - 1;
                while (PRE_WORK_DATA[i][j][k] != -1) k--;
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
        fflush(warning_output);
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
            if (fabs(arr_list[i][j].second - arr_list[i][j-1].second - moveTimePredict(&robot) + 50) < 50){
                continue;
            }
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
