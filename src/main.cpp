#include <iostream>
#include <stdio.h>
#include <cassert>
#include <ctime>
#include <cmath>
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
    int buy_expect[52] = {};
    int sell_expect[52][10] = {};
    vector<pair<double, pair<Robot*, Studio*> > > work_list;
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
        for (auto studio = studio_list.begin(); studio != studio_list.end(); studio++){
            double dist = abs(robot->position - studio->position) + robot->id*1000;
            work_list.push_back(make_pair(dist, make_pair(&(*robot), &(*studio))));
        }
    }
    sort(work_list.begin(), work_list.end());
    for (auto work = work_list.begin(); work != work_list.end(); work++){
        Robot* robot = work->second.first;
        Studio* studio = work->second.second;
        if (robot->task_now != Task::NONE){
            continue;
        }
        if (robot->item){
            int item_id = 0;
            for (int x=(robot->item>>1); x; x>>=1) item_id++;
            if ((MATERIAL[studio->type]&robot->item) == 0){
                continue;
            }
            double dist = abs(robot->position - studio->position);

            int space_left = (studio->item&robot->item) == 0;
            space_left += (studio->item==MATERIAL[studio->type]) && (studio->time_left!=-1 && studio->finish==0 && studio->time_left < dist/6*FRAME_PRE_SEC);
            space_left -= sell_expect[studio->id][item_id];
            if (space_left > 0){
                robot->dispatch(studio);
                sell_expect[studio->id][item_id]++;
            }
        }else{
            if (studio->type > 7){
                continue;
            }
            double dist = abs(robot->position - studio->position);
            int item_left = int(studio->finish) + int(studio->time_left!=-1 && studio->time_left < dist/6*FRAME_PRE_SEC);
            item_left -= buy_expect[studio->id];
            if (item_left > 0 && item_require[studio->type] > 0){
                robot->dispatch(studio);
                buy_expect[studio->id]++;
                item_require[studio->type]--;
            }
        }
    }


    for (auto robot_A = robot_list.begin(); robot_A != robot_list.end(); robot_A++){
        for (auto robot_B = robot_A + 1; robot_B != robot_list.end(); robot_B++){
            for (double time = 0; time < 1; time += 0.1){
                Point next_delta = (robot_A->position - robot_B->position) + (robot_A->velocity - robot_B->velocity) * time;
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
                robot_A->setAngleV(robot_A->angle_v - flag_A*1.5);
                robot_B->setAngleV(robot_B->angle_v - flag_B*1.5);
                break;
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
