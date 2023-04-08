#include "game.h"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <ctime>
#include "map.h"

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

//价值函数，还很粗糙
void Game::calcValue(){
    if (frameID > 8800){
        value = money*1000;
        return;
    }
    value = money - frameID;
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->item){
            int item_id = 0;
            for (int x=(robot->item>>1); x; x>>=1) item_id++;
            robot->flushTimeS(frameID);
            value += VALUE[item_id] * robot->time_s * robot->collision_s;
        }
        if (robot->target != -1){
            value -= abs(robot->position - studio_list[robot->target].position);
        }else{
            value -= 100;
        }
    }
    for (auto studio = studio_list.begin(); studio != studio_list.end(); studio++){
        if (studio->finish && studio->type > 3){
            value += (VALUE[studio->type] - COST[studio->type]) * 0.6;
        }
        if (studio->time_left != -1 && studio->type < 8 && studio->type > 3){
            value += (VALUE[studio->type] - COST[studio->type]) * 0.3;
        }
    }
}


//跟passTime结合使用，找到下一个关键帧
int Game::nextTimeStep(){
    int min_time_cost = 9000 - frameID;
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->task_now == Task::NONE){
            return 0;
        }
    }
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->task_now == Task::STOP || robot->task_now == Task::WAIT){
            continue;
        }
        Studio* studio = &(studio_list[robot->target]);
        if (robot->task_now == Task::BUY){
            int time_cost = max(0, moveTimePredict(&(*robot)) - (frameID - robot->data_frameID));
            if (studio->finish == 0){
                if (studio->time_left != -1){
                    time_cost = max(time_cost, studio->time_left);
                }else{
                    time_cost = INF;
                }
            }
            min_time_cost = min(min_time_cost, time_cost);
        }else if (robot->task_now == Task::SELL){
            int time_cost = max(0, moveTimePredict(&(*robot)) - (frameID - robot->data_frameID));
            if ((studio->item & robot->item) != 0){
                if (MATERIAL[studio->type] == studio->item && studio->time_left != -1 && studio->finish == 0){
                    time_cost = max(time_cost, studio->time_left);
                }else{
                    time_cost = INF;
                }
            }
            min_time_cost = min(min_time_cost, time_cost);
        }
    }
    return min_time_cost;
}

//不考虑物理部分的简化版模拟器，用来搜索调度
void Game::passTime(int time){
    time = min(time, 9000 - frameID);
    frameID += time;
    for (auto studio = studio_list.begin(); studio != studio_list.end(); studio++){
        int time_leaf = time;
        for (int i = 0; i < 2; i++){
            studio->update();
            if (studio->time_left == -1){
                break;
            }
            if (studio->time_left > time_leaf){
                studio->time_left -= time_leaf;
                time_leaf = 0;
            }else{
                time_leaf -= studio->time_left;
                studio->time_left = 0;
            }
            studio->update();
        }
    }

    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->target == -1){
            robot->data_frameID = frameID;
            continue;
        }
        Studio* studio = &studio_list[robot->target];
        bool arrive = absManhattan(robot->position - studio->position) < EPS;
        if ((!arrive) && (frameID - robot->data_frameID) >= moveTimePredict(&*robot)){
            Point delta = studio->position - robot->position;
            robot->angle = atan2(delta.y, delta.x);
            robot->position = studio->position;
            robot->data_frameID = frameID;
            arrive = true;
        }
        if (arrive){
            robot->studio_id = studio->id;
            money += robot->update(studio_list, frameID);
        }
    }
    if (time > 0){
        for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
            if (robot->task_now == Task::WAIT){
                robot->task_now = Task::NONE;
            }
        }
    }
}

//全量物理模拟器，用于碰撞检测的，效率比较低
void Game::physicalSimulation(stack<pair<int, int> > robot_target_stack[4]){
    int period_frame = 1;
    double period_time = 1.*period_frame / FRAME_PRE_SEC;

    frameID += period_frame;

    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->target != -1){
            Studio* studio = &studio_list[robot->target];
            studio->update();
            if (studio->time_left > 1){
                studio->time_left--;
            }
            studio->update();
            if (abs(robot->position - studio->position) < 0.4){
                robot->studio_id = robot->target;
                money += robot->update(studio_list, frameID);
            }
        }
        if (robot->task_now == Task::WAIT){
            robot->task_now = Task::NONE;
        }
        if (robot->task_now != Task::NONE){
            robot->goToTargetStudio(&(studio_list[robot->target]));
        }else if (robot_target_stack[robot->id].empty() || robot_target_stack[robot->id].top().first==-1){
            robot->task_now = Task::WAIT;
        }else{
            int delta_money = -1;
            while(delta_money != 0 && robot_target_stack[robot->id].empty() == false && robot_target_stack[robot->id].top().first != -1){
                int robot_target = robot_target_stack[robot->id].top().first;
                robot->dispatch(&(studio_list[robot_target]), robot_target_stack[robot->id].top().second);
                robot_target_stack[robot->id].pop();
                delta_money = robot->update(studio_list, frameID);
                money += delta_money;
            }
        }
    }
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->additional_frame_num){
            robot->setAngleV(robot->additional_angle_v);
            robot->setVelocity(robot->additional_pos_v);
            robot->additional_frame_num -= 1;
        }
        robot->physicalUpdate(period_time);
    }
}

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
    const double item_value[10]={0, 0, 0, 0, 6, 6, 6, 10, 0, 0};
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
                if (dist/5*FRAME_PRE_SEC + frameID > 15000){
                    value -= 1e6;
                }
                value -= ((studio->type > 8) && (robot->item != (1<<7)))*10000;
                int item_num = 0;
                for (int x = studio->item;x;x>>=1) item_num += x&1;
                value += item_num * 2 * item_value[studio->type];
                if ((robot->item|studio->item) == MATERIAL[studio->type]){
                    value += 4 * item_value[studio->type];
                }
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
                if (dist/5*FRAME_PRE_SEC + frameID > 15000){
                    continue;
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