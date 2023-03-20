#include "robot.h"
#include "main.h"
#include <stdio.h>
#include <cmath>

Robot::Robot(int _id, Point _position)
:id(_id),position(_position),task_now(NONE),target(-1){
    data_frameID = 0;
    angle = 0;
    item = 0;
    angle_v = 0;
    time_s = 0;
    collision_s = 0;
};

void Robot::readFromString(char input[])
{
    sscanf(input, "%d %d %lf %lf %lf %lf %lf %lf %lf %lf",
            &studio_id, &item, &time_s, &collision_s, &angle_v, &velocity.x, &velocity.y, &angle, &position.x, &position.y);

    if (item > 0){
        item = 1<<item;
    }
    data_frameID = frameID;
}

void Robot::outputToString(char output[]){
    sprintf(output, "Robot id: %d \nstudio_id: %d item: %d \ntime_s: %.3lf collision_s: %.3lf \nangle: %.3lf angle_v: %.3lf \nposition: (%.3lf, %.3lf) velocity: (%.3lf, %.3lf)\ntask: %d target: %d\n\n\n",
            id, studio_id, item, time_s, collision_s, angle, angle_v, position.x, position.y, velocity.x, velocity.y, task_now, target);
}


void Robot::goToTargetStudio(Studio* studio){
    Point delta = studio->position - position;
    double target_angle = atan2(delta.y, delta.x);
    double angle_delta = angleAdjust(target_angle - angle);
    setAngleV(angle_delta * 5);

    double v = 0;
    if (abs(angle_delta) < 1){
        v = 6;
        if (abs(delta) < 1){
            v = 1.5;
        }
        if (abs(delta) < 0.4){
            v = 0;
        }
        if ((studio->item&item)){
            if (abs(delta) <= 2){
                v = 0;
            }
            if (abs(delta) < 0.5){
                v = -2;
            }
        }
    }

    setVelocity(v);
}

void Robot::stop(){
    angle_v = 0;
    double v = 0;
    velocity = Point(sin(angle), cos(angle))*v;
    printf("rotate %d %lf\n", id, angle_v);
    printf("forward %d %lf\n", id, v);
}

int Robot::buy(vector<Studio> &studio_list, int frameID, bool output){
    if (item != 0){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d try to sell buy while holding something! Item: %d\n", id, item);
#endif
        return 0;
    }
    if (studio_id == -1){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d try to buy without studio nearby!\n", id);
#endif
        return 0;
    }

    if (studio_list[studio_id].finish == 0){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d try to buy with studio working!\n", id);
#endif
        return 0;
    }

    int money = 0;
    if (output){
        printf("buy %d\n", id);
    }
    time_s = 1;
    collision_s = 1;
    item = PRODUCT[studio_list[studio_id].type];
    pick_up_time = frameID;
    studio_list[studio_id].finish = 0;
    for (int i = 1; i <= 7; i++){
        if ((item>>i)&1){
            money -= COST[i];
        }
    }
    return money;
}

int Robot::sell(vector<Studio> &studio_list, int frameID, bool output){
    if (item == 0){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d try to sell without holding anything!\n", id);
#endif
        return 0;
    }
    if (studio_id == -1){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d try to sell without studio nearby!\n", id);
#endif
        return 0;
    }

    int item_sell = MATERIAL[studio_list[studio_id].type]&item;
    if (item_sell == 0 || (item_sell&studio_list[studio_id].item) != 0){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d try to sell with wrong studio! Studio id: %d, Item: %d, Studio item: %d\n", id, studio_id, item, studio_list[studio_id].item);
#endif
        return 0;
    }

    int money = 0;
    if (output){
        printf("sell %d\n", id);
    }
    item ^= item_sell;
    studio_list[studio_id].item ^= item_sell;
    
    int item_id = 0;
    for (int x=(item_sell>>1); x; x>>=1) item_id++;
    flushTimeS(frameID);
    money += int(VALUE[item_id] * 1 * collision_s);
    return money;
}
void Robot::destroy(){
    if (item == 0){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d try to destroy without holding anything!\n", id);
#endif
        return;
    }

    printf("destroy %d\n", id);
    item = 0;
}


int Robot::update(vector<Studio> &studio_list, int frameID, bool output){
    int money = 0;
    if (target == -1){
        return money;
    }
    Studio* target_studio = &(studio_list[target]);
    if (task_now == Task::BUY && target_studio->finish == 0){
        return money;
    }
    if (task_now == Task::BUY && studio_id == target_studio->id){
        money += buy(studio_list, frameID, output);
        if (money){
            task_now = Task::NONE;
            target = -1;
        }
        return money;
    }
    if (task_now == Task::SELL && (target_studio->item & item)){
        return money;
    }
    if (task_now == Task::SELL && studio_id == target_studio->id){
        money += sell(studio_list, frameID, output);
        if (money){
            task_now = Task::NONE;
            target = -1;
        }
        return money;
    }
    return money;
}

void Robot::dispatch(Studio* studio, bool output){
    if (item){
        task_now = Task::SELL;
    }else{
        task_now = Task::BUY;
    }
    target = studio->id;
    if (output){
        goToTargetStudio(studio);
    }
}


void Robot::setAngleV(double _angle_v){
    angle_v = _angle_v;
    if (angle_v > M_PI){
        angle_v = M_PI;
    }
    if (angle_v < -M_PI){
        angle_v = -M_PI;
    }
    printf("rotate %d %lf\n", id, angle_v);
}
void Robot::setVelocity(double v){
    if (v >= 6){
        v = 6;
    }
    if (v <= -2){
        v = -2;
    }
    velocity = Point(cos(angle), sin(angle))*v;
    printf("forward %d %lf\n", id, v);
}

double Robot::getRadius(){
    return item? 0.53: 0.45;
}


void Robot::flushTimeS(int frameID){
    double rate = 1 - 1.*(frameID - pick_up_time) / 9000;
    //time_s = 0.8 + 0.2*(1-sqrt(1-rate*rate));
}