#include "game.h"
#include <algorithm>
#include <cassert>

int moveTimePredict(Robot* robot){
    Studio* studio = &(studio_list[robot->target]);
    Point delta = robot->position - studio->position;
    double dist = abs(delta);
    double angle = atan2(delta.y, delta.x);
    double time = dist/6 + abs(angleAdjust(angle - robot->angle))/M_PI*2;
    return int((time+2) * FRAME_PRE_SEC); //To be adjust
}

Game::Game(vector<Studio> &_studio_list, vector<Robot> &_robot_list, int _frameID, int _money)
    :studio_list(_studio_list), robot_list(_robot_list), frameID(_frameID), money(_money){};

void Game::calcValue(){
    value = money;
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->item){
            int item_id = 0;
            for (int x=(robot->item>>1); x; x>>=1) item_id++;
            robot->flushTimeS(frameID);
            value += VALUE[item_id] * robot->time_s * robot->collision_s * 0.8;
        }
        if (robot->target != -1){
            value -= abs(robot->position - studio_list[robot->target].position);
        }else{
            value -= 100;
        }
    }
    for (auto studio = studio_list.begin(); studio != studio_list.end(); studio++){
        if (studio->finish){
            value += (VALUE[studio->type] - COST[studio->type]) * 0.1;
        }
        if (studio->time_left != -1 && studio->type < 8 && studio->type > 3){
            value += (VALUE[studio->type] - COST[studio->type]) * 10;
        }
    }
}
int Game::nextTimeStep(){
    int min_time_cost = 9000 - frameID;
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->task_now == Task::NONE){
            return 0;
        }
    }
    for (auto studio = studio_list.begin(); studio != studio_list.end(); studio++){
        if (studio->time_left > 0){
            min_time_cost = min(min_time_cost, studio->time_left);
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
            studio->update();
        }
    }
}

void Game::greedyWork(double value_list[4][50]){
    int buy_expect[52] = {};
    int sell_expect[52][10] = {};
    vector<pair<double, pair<Robot*, Studio*> > > work_list;
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        robot->update(studio_list, frameID);
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
        if (robot->task_now == Task::BUY){
            buy_expect[robot->target]++;
            item_require[studio_list[robot->target].type]--;
        }else if (robot->task_now == Task::SELL){
            int item_id = 0;
            for (int x=(robot->item>>1); x; x>>=1) item_id++;
            sell_expect[robot->target][item_id]++;
        }
    }

    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->task_now == Task::WAIT){
            robot->task_now = Task::NONE;
        }
        if (robot->task_now != Task::NONE){
            continue;
        }
        for (auto studio = studio_list.begin(); studio != studio_list.end(); studio++){
            double value = -abs(robot->position - studio->position) - robot->id*1000;
            work_list.push_back(make_pair(-value, make_pair(&(*robot), &(*studio))));
        }
    }
    if (value_list == NULL){
        sort(work_list.begin(), work_list.end());
    }
    for (auto work = work_list.begin(); work != work_list.end(); work++){
        Robot* robot = work->second.first;
        Studio* studio = work->second.second;
        if (value_list == NULL && robot->task_now != Task::NONE){
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
            space_left += (studio->item==MATERIAL[studio->type]) && (studio->time_left!=-1 && studio->finish==0 && studio->time_left < dist/6*FRAME_PRE_SEC + 50);
            space_left -= sell_expect[studio->id][item_id];
            if (space_left > 0){
                if (value_list){
                    value_list[robot->id][studio->id] = -(work->first);
                }else{
                    robot->dispatch(studio);
                    sell_expect[studio->id][item_id]++;
                }
            }
        }else{
            if (studio->type > 7){
                continue;
            }
            double dist = abs(robot->position - studio->position);
            int item_left = int(studio->finish) + int(studio->time_left!=-1 && studio->time_left < dist/6*FRAME_PRE_SEC);
            item_left -= buy_expect[studio->id];
            if (item_left > 0 && item_require[studio->type] > 0){
                if (value_list){
                    value_list[robot->id][studio->id] = -(work->first);
                }else{
                    robot->dispatch(studio);
                    buy_expect[studio->id]++;
                    item_require[studio->type]--;
                }
            }
        }
    }
    for (auto robot = robot_list.begin(); robot != robot_list.end(); robot++){
        if (robot->task_now == Task::NONE && value_list==NULL){
            robot->task_now = Task::WAIT;
        }
    }
}

int Game::MCTS(int robot_id){
    assert(robot_list[robot_id].task_now == Task::NONE);
    int max_value = -INF;
    int action;
    
    double value_list[4][50];
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 50; j++){
            value_list[i][j] = -INF;
        }
    }
    
    greedyWork(value_list);
    double tmp[50];
    memcpy(tmp, value_list[robot_id], sizeof(tmp));
    sort(tmp, tmp+studio_list.size());
    for (int j = 0; j < studio_list.size(); j++){
        if (value_list[robot_id][j] == -INF){
            continue;
        }
        if (value_list[robot_id][j] <= tmp[studio_list.size() - 1 - 3]){
            continue;
        } 
        Game g(*this);
        g.robot_list[robot_id].dispatch(&(g.studio_list[j]));
        while (g.frameID < min(9000, frameID + 1000)){
            g.greedyWork();
            g.passTime(g.nextTimeStep());
        }
        value_list[robot_id][j] = g.money * 0.1;

        #ifdef DEBUG_MODE
            fprintf(warning_output, "Frame: %d, ID: %d, Target: %d, Money: %d %d, Value: %lf\n", frameID, robot_id, j, g.money, g.money, value_list[robot_id][j]);
            fflush(warning_output);
        #endif

        if (max_value < value_list[robot_id][j]){
            max_value = value_list[robot_id][j];
            action = j;
        }
    }
    return action;
}