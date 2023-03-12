#include <iostream>
#include <stdio.h>
#include <cassert>
#include <ctime>
#include <cstring>
#include <algorithm>
#include "main.h"

vector<Studio> studio_list;
vector<Robot> robot_list;
vector<Studio*> studio_dict[10];
FILE* warning_output;
int money = 200000;
int frameID;
char map[110][110];
void readUntilOK() {
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

void work(){
    int buy_expect[100] = {};
    int sell_expect[100] = {};
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        robot->update();
    }
    int item_require[8]={};
    
    for (auto studio = studio_list.begin(); studio != studio_list.end(); studio++){
        for (int i = 1; i <= 7; i++){
            if (((MATERIAL[studio->type]^studio->item)>>i)&1){
                item_require[i]++;
            }
        }
    }
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        for (int i = 1; i <= 7; i++){
            if ((robot->item>>i)&1)
            item_require[i]--;
        }
    }

    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        robot->task_now = Task::NONE;

        if (robot->item){
            double dist = INF;
            Studio* target = NULL;
            for (int i = 0; i < 10; i++){
                if ((MATERIAL[i]&robot->item) == 0){
                    continue;
                }
                for (int j = 0; j < studio_dict[i].size() && robot->item; j++){
                    Studio* studio = studio_dict[i][j];
                    if ((studio->item | sell_expect[studio->id]) & robot->item){
                        continue;
                    }
                    double dist_tmp = abs2(robot->position - studio->position);
                    if (dist_tmp < dist){
                        target = studio;
                        dist = dist_tmp;
                    }
                }
            }
            if (target){
                robot->dispatch(Task::SELL, target);
                sell_expect[target->id] ^= robot->item;
            }
        }else{
            double dist = INF;
            Studio* target = NULL;
            for (int i = 7; i > 0; i--){
                if (item_require[i] <= 0){
                    continue;
                }

                for (int j = 0; j < studio_dict[i].size() && (!robot->item); j++){
                    Studio* studio = studio_dict[i][j];
                    if (studio->finish==0 || buy_expect[studio->id]==1){
                        continue;
                    }
                    double dist_tmp = abs2(robot->position - studio->position);
                    if (dist_tmp < dist){
                        target = studio;
                        dist = dist_tmp;
                    }
                }
            }
            if (target){
                robot->dispatch(Task::BUY, target);
                buy_expect[target->id] = 1;
                item_require[target->type]--;
            }
        }
    }
}

int main() {
#ifdef DEBUG_MODE
    warning_output = fopen("warning.txt", "w");
#endif

    readMap();
    puts("OK");
    fflush(stdout);
    while (scanf("%d", &frameID) != EOF) {
        readUntilOK();
        printf("%d\n", frameID);

        work();
#ifdef DEBUG_MODE
        fprintf(warning_output, "Start frame ID: %d, money: %d, clock: %.6lfs %lf %lf\n", frameID, money, double(clock())/CLOCKS_PER_SEC, robot_list[0].position.x, robot_list[0].position.y);
#endif
        printf("OK\n", frameID);

        stateOutput();
        fflush(stdout);
#ifdef DEBUG_MODE
        fflush(warning_output);
#endif
    }
    
#ifdef DEBUG_MODE
    fclose(warning_output);
#endif
    return 0;
}
