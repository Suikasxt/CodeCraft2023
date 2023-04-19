#include "game.h"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <ctime>
#include "map.h"

//orangesheee 修改
const int MAX_FRAME=12000;

//给简化版模拟器也就是passTime函数用，预测一个机器人要多长时间到达目标工作台
int moveTimePredict(Robot* robot){
    Studio* studio = &(studio_list[robot->target]);
    Point delta = studio->position - robot->position;
    double dist = abs(delta);
    double angle = atan2(delta.y, delta.x);
    double time = dist/6;
    double angle_delta = fabs(angleAdjust(angle - robot->angle));
    if (angle_delta > 1){
        time += (angle_delta-1)/M_PI;
        angle_delta = 1;
    }
    return int((time + angle_delta*0.4) * FRAME_PRE_SEC); //To be adjust
}

Game::Game(vector<Studio> &_studio_list, vector<Robot> &_robot_list, int _frameID, int _money)
    :studio_list(_studio_list), robot_list(_robot_list), frameID(_frameID), money(_money){};

extern int time_count;
//贪心策略
void Game::greedyWork(double value_list[4][50]){
    int buy_expect[52] = {};
    int sell_expect[52][10] = {};
    vector<pair<double, pair<Robot*, pair<Studio*, Studio*> > > > work_list;
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        robot->update(studio_list, frameID);
    }
    double item_require[8]={};
    double item_value[10]={0, 0, 0, 0, 6, 6, 6, 10, 0, 0};
    for (auto studio = studio_list.begin(); studio != studio_list.end(); studio++){
        for (int i = 1; i <= 7; i++){
            if (((MATERIAL[studio->type]^studio->item)>>i)&1){
                item_require[i]++;
            }
        }
        if (studio->finish){
            item_require[studio->type]--;
        }
        if (studio->time_left > -1){
            item_require[studio->type]--;
        }
    }
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        for (int i = 1; i <= 7; i++){
            if ((robot->item>>i)&1)
            item_require[i]--;
        }
    }
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->task_now == Task::BUY){
            buy_expect[robot->target]++;
            sell_expect[robot->next_target][studio_list[robot->target].type]++;
        }else if (robot->task_now == Task::SELL){
            int item_id = 0;
            for (int x=(robot->item>>1); x; x>>=1) item_id++;
            sell_expect[robot->target][item_id]++;
        }
    }

    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        pair<int, int> coord_robot = Continuous2DiscreteRound(robot->position);
        
        if (value_list==NULL && robot->task_now == Task::WAIT){
            robot->task_now = Task::NONE;
        }
        if (robot->task_now != Task::NONE){
            continue;
        }
        if (robot->item){
            for (auto studio = studio_list.begin(); studio != studio_list.end(); studio++){
                if ((MATERIAL[studio->type]&robot->item) == 0){
                    continue;
                }
                if (map_dist[1][studio->id][coord_robot.first][coord_robot.second] >= INF){
                    continue;
                }
                
                double dist = map_dist[1][studio->id][coord_robot.first][coord_robot.second];
                double value = -dist;
                value += 1e7;
                if (dist/5*FRAME_PRE_SEC + frameID > MAX_FRAME){
                    value -= 1e6;
                }
                value -= ((studio->type > 8) && (robot->item != (1<<7)))*10000;
                int item_num = 0;
                for (int x = studio->item;x;x>>=1) item_num += x&1;
                value += item_num * 2 * item_value[studio->type];
                if ((robot->item|studio->item) == MATERIAL[studio->type]){
                    value += 4 * item_value[studio->type];
                }
                value += item_value[studio->type] * 10;
                value += item_require[studio->type] * item_value[studio->type] * 5;
                work_list.push_back(make_pair(-value, make_pair(&(*robot), make_pair(&(*studio), &(*studio)))));
            }
        }else{
            for (auto studio = studio_list.begin(); studio != studio_list.end(); studio++)
            for (auto next_studio = studio_list.begin(); next_studio != studio_list.end(); next_studio++){
                if (studio->type > 7){
                    continue;
                }
                if ((MATERIAL[next_studio->type]&PRODUCT[studio->type]) == 0){
                    continue;
                }
                if (map_dist[0][studio->id][coord_robot.first][coord_robot.second] >= INF){
                    continue;
                }
                if (studio_dist[studio->id][next_studio->id] >= INF){
                    continue;
                }
                
                double dist = map_dist[0][studio->id][coord_robot.first][coord_robot.second] + studio_dist[studio->id][next_studio->id];
                double value = -dist;
                if (dist/5*FRAME_PRE_SEC + frameID > MAX_FRAME){
                    continue;
                }
                dist += studio_dist[studio->id][next_studio->id];
                double value = -dist;
                for (int j = 1; j <= 6; j++)
                if (sell_expect[studio->id][j]){
                    value -= 1e4;
                }
                value -= ((next_studio->type > 8) && (studio->type != 7))*10000;
                value += (studio->type == 7)*1000;
                value += (studio->type >= 4)*500;
                if (studio->finish && studio->item == MATERIAL[studio->type]){
                    value += 300;
                }
                int item_num = 0;
                for (int x = studio->item;x;x>>=1) item_num += x&1;
                value += item_num * 2 * item_value[studio->type];
                if ((robot->item|studio->item) == MATERIAL[studio->type]){
                    value += 4 * item_value[studio->type];
                }
                value += item_value[studio->type] * 10;
                value += item_value[next_studio->type] * 10;
                value += item_require[next_studio->type] * item_value[next_studio->type] * 5;
                work_list.push_back(make_pair(-value, make_pair(&(*robot), make_pair(&(*studio), &(*next_studio)))));
            }
        }
    }
    if (value_list == NULL){
        sort(work_list.begin(), work_list.end());
    }
    for (auto work = work_list.begin(); work != work_list.end(); work++){
        Robot* robot = work->second.first;
        Studio* studio = work->second.second.first;
        Studio* next_studio = work->second.second.second;
        pair<int, int> coord_robot = Continuous2DiscreteRound(robot->position);
        if (value_list == NULL && robot->task_now != Task::NONE){
            continue;
        }
        if (robot->item){
            int item_id = 0;
            for (int x=(robot->item>>1); x; x>>=1) item_id++;
            double dist = map_dist[0][studio->id][coord_robot.first][coord_robot.second];

            double space_left = (studio->item&robot->item) == 0;
            space_left += (studio->item==MATERIAL[studio->type]) && (studio->time_left!=-1 && (next_studio->finish==0 || buy_expect[next_studio->id]) && studio->time_left < dist/5*FRAME_PRE_SEC+50);
            space_left -= sell_expect[studio->id][item_id];
            if (space_left > 0 && value_list == NULL){
                robot->dispatch(studio, -1);
                sell_expect[studio->id][item_id]++;
            }
            if (value_list){
                value_list[robot->id][studio->id] = -(work->first) + fmin(space_left - 1, 0)*10000;
            }
        }else{
            pair<int, int> coord_studio = Continuous2DiscreteRound(studio->position);
            double dist = map_dist[0][studio->id][coord_robot.first][coord_robot.second];
            double item_left = int(studio->finish) + int(studio->time_left!=-1 && studio->time_left < dist/5*FRAME_PRE_SEC+50);
            item_left -= buy_expect[studio->id];
            if (studio->type <= 3){
                item_left += 1;
            }
            
            dist += studio_dist[studio->id][next_studio->id];
            double space_left = (next_studio->item&PRODUCT[studio->type]) == 0;
            space_left += (next_studio->item==MATERIAL[next_studio->type]) && (next_studio->time_left!=-1 && (next_studio->finish==0 || buy_expect[next_studio->id]) && next_studio->time_left < dist/5*FRAME_PRE_SEC+50);
            space_left -= sell_expect[next_studio->id][studio->type];

            if (item_left > 0 && space_left > 0 && value_list == NULL){
                robot->dispatch(studio, -1);
                robot->next_target = next_studio->id;
                robot->additional_target_position = studio_dist_addition[studio->id][next_studio->id];
                buy_expect[studio->id]++;
                sell_expect[next_studio->id][studio->type]++;
            }
            if (value_list){
                value_list[robot->id][studio->id] = -(work->first) + fmin(item_left - 1, 0)*10000 + fmin(space_left - 1, 0)*10000;
            }
        }
    }
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->task_now == Task::NONE && value_list==NULL){
            robot->task_now = Task::WAIT;
        }
    }
}