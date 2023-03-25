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
#include <string.h>
#include <stdlib.h>

vector<Studio> studio_list;
vector<Robot> robot_list;
vector<Studio*> studio_dict[10];
FILE* warning_output;
int money = 200000;
int frameID = 0;
char map[110][110];
int robot_target[4];
stack<pair<int, int> > robot_target_stack[4];
vector<pair<int, int>> robot_target_real[4];
vector<pair<int, int> > arr_list[4];
bool pre_work = false;
int map_num = 0;
//本地计算的的时候主要关注下面每个多长时间重新计算，以及每次计算用多大的beam，上面search的部分扩展的宽度也是可以调的
int interval = 500;
int beam_width = 100;
//每次选贪心策略分数最高的K个来扩展，也是计算的时候可以调的参数之一
int search_width_K = 1; //2~5

int time_count = 0;
int total_time_count = 0;
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
    
    total_time_count -= clock();

    Heap heap(width);
    int max_time = min(frameID + time, 9000);
    vector<UpdateRoad> road;
    Game* g = new Game(studio_list, robot_list, frameID, money);
    g->road_id = -1;
    g->passTime(0);
    heap.push(g);
    double max_value = -INF;
    int res_road_id = -1;
    while (heap.size){
        Game* g = new Game(*heap.top());
        //fprintf(stderr, "%d\n", g->frameID);
        if (g->money > max_value && g->road_id != -1){
            max_value = g->money;
            res_road_id = g->road_id;
        }
        heap.pop(1, 0);
        int action_num[4] = {-1, -1, -1, -1};
        
        for (int i = 0; i < 4; i++){
            if (g->robot_list[i].target != -1){
                action_num[i] = g->studio_list[g->robot_list[i].target].action_num;
            }
        }
        g->passTime(g->nextTimeStep());
        if (g->frameID > max_time){
            delete g;
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
            memcpy(tmp, value_list[i], sizeof(tmp));
            sort(tmp, tmp+studio_list.size());
            for (int j = 0; j < g->studio_list.size(); j++){
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
                    road.push_back(UpdateRoad(g->road_id, i, j, action_num[i]));
                }
            }
            if (robot->item == 0){
                Game* new_g = new Game(*g);
                new_g->robot_list[i].task_now = Task::STOP;
                new_g->robot_list[i].target = -1;
                if (heap.push(new_g)){
                    new_g->road_id = road.size();
                    road.push_back(UpdateRoad(g->road_id, i, -1, action_num[i]));
                }
            }
            break;
        }
        delete g;
    }
    memset(robot_target, -1, sizeof(robot_target));
    int last_action_num[4] = {-1, -1, -1, -1};
    for (int i = 0; i < 4; i++){
        while (!robot_target_stack[i].empty()) robot_target_stack[i].pop();
    }
    while (res_road_id!=-1){
        robot_target[road[res_road_id].robot_id] = road[res_road_id].target_id;
        if (road[res_road_id].target_id!=-1){
            robot_target_stack[road[res_road_id].robot_id].push(make_pair(road[res_road_id].target_id, last_action_num[road[res_road_id].robot_id]));
        }
        last_action_num[road[res_road_id].robot_id] = road[res_road_id].action_num;
        res_road_id = road[res_road_id].pre_id;
    }
#ifdef DEBUG_MODE
    for (int i = 0; i < 4; i++){
        stack<pair<int, int> > tmp(robot_target_stack[i]);
        fprintf(stderr, "{");
        while (!tmp.empty()){
            fprintf(stderr, "{%d, %d}, ", tmp.top().first, tmp.top().second);
            tmp.pop();
        } 
        fprintf(stderr, "},\n");
    }
#endif
    total_time_count += clock();
    fprintf(stderr, "{%d, %d, %d}\n", studio_list.size(), studio_list[0].type, studio_list[1].type);
    fprintf(stderr, "FrameID %d Except money: %lf time: %lf %lf\n", frameID, max_value, 1.*total_time_count/CLOCKS_PER_SEC, 1.*time_count/CLOCKS_PER_SEC);
}

int last_design = -INF;
void work(){
    bool redesign = false;
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->task_now == Task::WAIT){
            robot->task_now = Task::NONE;
        }
        if (robot->target != -1){
            int real_action_num = studio_list[robot->target].action_num;
            int delta_money = robot->update(studio_list, frameID, true);
            if (delta_money){
                (robot_target_real[robot->id].end()-1)->second = real_action_num;
                money += delta_money;
            }
        }
        if (robot->task_now == Task::NONE && frameID > last_design + interval){
            redesign = true;
        }
    }
    if (redesign){
        for (int i = 0; i < 4; i++){
            robot_list[i].target_action_num = -1;
        }
        search(beam_width, 9000);
        last_design = frameID;
    }

    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->task_now == Task::STOP){
            continue;
        }
        if (robot->task_now != Task::NONE){
            robot->goToTargetStudio(&(studio_list[robot->target]), true);
        }else if (robot_target_stack[robot->id].empty() || robot_target_stack[robot->id].top().first==-1){
            robot->task_now = Task::WAIT;
        }else{
            if (robot_target_stack[robot->id].empty() || robot_target_stack[robot->id].top().first==-1){
                continue;
            }
            int delta_money = 0;
            do{
                int action_num = robot_target_stack[robot->id].top().second;
                robot_target[robot->id] = robot_target_stack[robot->id].top().first;
                robot_target_real[robot->id].push_back(make_pair(robot_target[robot->id], action_num));
                robot_target_stack[robot->id].pop();
                robot->dispatch(&(studio_list[robot_target[robot->id]]), action_num, true);
                //break;//Not sure
                int real_action_num = studio_list[robot->target].action_num;
                delta_money = robot->update(studio_list, frameID, true);
                if (delta_money){
                    (robot_target_real[robot->id].end()-1)->second = real_action_num;
                    money += delta_money;
                }
            }while(delta_money != 0 && robot_target_stack[robot->id].empty() == false && robot_target_stack[robot->id].top().first != -1);
            //fprintf(stderr, "%d dispatch %d %d action_num: %d item: %d Money: %d\n", frameID, robot->id, robot->target, robot->target_action_num, robot->item, money);
        }
    }
    
    //碰撞检测
    stack<pair<int, int> > rt[4];
    double gap = 0.01;
    bool collision[4] = {0, 0, 0, 0};
    pair<double, int> dist_2_target_list[4];
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->target == -1){
            dist_2_target_list[robot->id] = make_pair(INF+1000, robot->id);
        }else if ((studio_list[robot->target].item&robot->item)!=0 || (robot->item==0 && studio_list[robot->target].finish==0) || studio_list[robot->target].action_num < robot->target_action_num){
            dist_2_target_list[robot->id] = make_pair(INF+abs(robot->position - studio_list[robot->target].position), robot->id);
        }else{
            dist_2_target_list[robot->id] = make_pair(abs(robot->position - studio_list[robot->target].position), robot->id);
        }
    }
    sort(dist_2_target_list, dist_2_target_list+4); //确定优先级
    int SIM_FRAME_NUM = 30;
    Game g(studio_list, robot_list, frameID, money);
    for (int i = 0; i < 4; i++){
        rt[i] = robot_target_stack[i];
    }
    for (int i = 0; i < SIM_FRAME_NUM; i++){
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
    for (int i = 3; i >= 0; i--)
    if (collision[dist_2_target_list[i].second]){
        robot_list[dist_2_target_list[i].second].additional_frame_num = 0;
    }
    for (int i = 3; i >= 0; i--){
        int robot_id = dist_2_target_list[i].second;
        //如果会碰撞
        if (collision[robot_id] == false){
            continue;
        }
        double min_dist_2_target = INF+EPS;
        //搜索解决方案
        for (int add_frame = 0; add_frame <= SIM_FRAME_NUM; add_frame += 3){
            for (double add_pos_v = -2; add_pos_v <= 6+EPS; add_pos_v+=4){
                for (double add_angle_v = -M_PI; add_angle_v <= M_PI+EPS; add_angle_v += M_PI_2){
                    if (add_frame == 0 && abs(add_angle_v) > EPS && add_pos_v!=-2){
                        continue;
                    }
                    for (int j = 0; j < 4; j++){
                        rt[j] = robot_target_stack[j];
                    }
                    Game g(studio_list, robot_list, frameID, money);
                    
                    g.robot_list[robot_id].additional_angle_v = add_angle_v;
                    g.robot_list[robot_id].additional_pos_v = add_pos_v;
                    g.robot_list[robot_id].additional_frame_num = add_frame;
                    bool collision = false;
                    double min_robot_dist = INF;
                    int f;
                    Point init_position = robot_list[robot_id].position;
                    double init_r = robot_list[robot_id].getRadius();
                    double last_dist[4];
                    Robot* robot_now = &(g.robot_list[robot_id]);
                    
                    for (auto robot = g.robot_list.begin(); robot != g.robot_list.end(); robot++){
                        last_dist[robot->id] = abs(robot->position - robot_now->position);
                    }
                    for (f = 0; f < SIM_FRAME_NUM && collision==false; f++){
                        g.physicalSimulation(rt);
                        //fprintf(stderr, "%d %d %d %d\n", f, robot_id, add_frame, g.robot_list[robot_id].additional_frame_num);
                        for (auto robot = g.robot_list.begin(); robot != g.robot_list.end(); robot++){
                            if (robot->id == robot_id){
                                continue;
                            }
                            double dist = abs(robot->position - robot_now->position);
                            min_robot_dist = min(min_robot_dist, dist);
                            if (dist < robot->getRadius() + robot_now->getRadius() + gap && dist < last_dist[robot->id]){
                                collision = true;
                            }
                            last_dist[robot->id] = dist;
                        }
                        double r = robot_now->getRadius();
                        if ((robot_now->position.x - r <  0 && robot_now->position.x < init_position.x) ||
                            (robot_now->position.y - r <  0 && robot_now->position.y < init_position.y) ||
                            (robot_now->position.x + r > 50 && robot_now->position.x > init_position.x) ||
                            (robot_now->position.y + r > 50 && robot_now->position.y > init_position.y)){
                            collision = true;
                        }
                    }
                    
                    double dist_2_target = 0;
#ifdef DEBUG_MODE
                    //fprintf(warning_output, "id: %d item: %d (%lf, %lf, %d) : %lf %lf\n", robot_id, robot_list[robot_id].item, add_angle_v, add_pos_v, add_frame, dist_2_target, min_dist_2_target);
                    //fflush(warning_output);
#endif
                    if (collision){
                        //continue;
                        dist_2_target += 2e8;
                        dist_2_target -= f*10;
                        dist_2_target -= min_robot_dist;
                    }else{
                        for (int k = 0; k < 4; k++){
                            dist_2_target += rt[k].size()*100;
                            int target = g.robot_list[k].target;
                            if (target == -1){
                                dist_2_target += 100;
                            }else if ((g.robot_list[k].item != 0 && (g.studio_list[target].item&g.robot_list[k].item)==0) || (g.robot_list[k].item == 0 && g.studio_list[target].finish == 0) || g.studio_list[target].action_num < g.robot_list[k].target_action_num){
                                Point delta = g.studio_list[target].position - g.robot_list[k].position;
                                dist_2_target += abs(delta);
                                dist_2_target += fabs(angleAdjust(atan2(delta.y, delta.x) - g.robot_list[k].angle))*0.1;
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
                collision[robot_id] = false;
                break;
            }
        }
#ifdef DEBUG_MODE
        fprintf(warning_output, "priority: %lf id: %d solution: (%lf %lf %d) : %lf\n", dist_2_target_list[i].first, robot_id, robot_list[robot_id].additional_angle_v, robot_list[robot_id].additional_pos_v, robot_list[robot_id].additional_frame_num, min_dist_2_target);
#endif
    }
    g = Game(studio_list, robot_list, frameID, money);
    for (int i = 0; i < 4; i++){
        rt[i] = robot_target_stack[i];
        collision[i] = false;
    }
    for (int i = 0; i < SIM_FRAME_NUM; i++){
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
    for (int i = 3; i >= 0; i--)
    if (collision[dist_2_target_list[i].second]){
        robot_list[dist_2_target_list[i].second].additional_frame_num = 0;
    }

    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->additional_frame_num){
            robot->setAngleV(robot->additional_angle_v, true);
            robot->setVelocity(robot->additional_pos_v, true);
            robot->additional_frame_num -= 1;
        }
    }
}

int main(int argc, char *argv[]) {
    bool calc_mode = false;
    for (int i = 0; i < argc; i++){
        if (strcmp(argv[i], "-c") == 0){
            calc_mode = true;
        }else
        if (strcmp(argv[i], "-bw") == 0){
            beam_width = atoi(argv[i+1]);
        }else
        if (strcmp(argv[i], "-k") == 0){
            search_width_K = atoi(argv[i+1]);
        }else
        if (strcmp(argv[i], "-i") == 0){
            interval = atoi(argv[i+1]);
        }
    }

    readMap();
    stateOutput();
    for (int i = 0; i < PRE_WORK_MAP_NUM; i++){
        if (studio_list.size() == MAP_FEATURE[i][0] && studio_list[0].type == MAP_FEATURE[i][1] && studio_list[1].type == MAP_FEATURE[i][2]){
            map_num = i+1;
            //这个开关用来切换计算模式还是推理模式，break开了就是本地计算，最后会打一个结果到warning.txt，从里面把数据贴到data.h就可以
#ifdef _LOCAL
            if (calc_mode){
                break;
            }
#endif
            //加载打表好的指令序列
            pre_work = true;
            for (int j = 0; j < 4; j++){
                while (!robot_target_stack[j].empty()) robot_target_stack[j].pop();
                int k = WORK_LIST_LENGTH - 1;
                while (PRE_WORK_DATA[i][j][k][0] != -1) k--;
                while (k>=0){
                    robot_target_stack[j].push(make_pair(PRE_WORK_DATA[i][j][k][0], PRE_WORK_DATA[i][j][k][1]));
                    k--;
                }
            }
            break;
        }
    }
    if (calc_mode){
        fprintf(stderr, "map %d bw %d k %d i %d\n", map_num, beam_width, search_width_K, interval);
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
    fprintf(stderr, "\n");
    if (calc_mode){
        fprintf(stderr, "map %d bw %d k %d i %d\n", map_num, beam_width, search_width_K, interval);
    }
    fprintf(stderr, "\n");
    fflush(stderr);
    for (int i = 0; i < 4; i++){
        stack<pair<int, int> > tmp(robot_target_stack[i]);
        fprintf(stderr, "{");
        for (auto j = robot_target_real[i].begin(); j != robot_target_real[i].end(); j++){
            fprintf(stderr, "{%d, %d}, ", j->first, j->second);
        } 
        fprintf(stderr, "-1},\n");
    }
    fprintf(stderr, "\n\n");
    fflush(stderr);
#endif
#ifdef DEBUG_MODE
    fflush(warning_output);
#endif
    
#ifdef DEBUG_MODE
    fclose(warning_output);
#endif
    return 0;
}
