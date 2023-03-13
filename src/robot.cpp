#include "robot.h"
#include "main.h"
#include <stdio.h>
#include <math.h>

Robot::Robot(int _id, Point _position)
:id(_id),position(position),task_now(NONE),target(NULL){};

void Robot::readFromString(char input[])
{
    sscanf(input, "%d %d %lf %lf %lf %lf %lf %lf %lf %lf",
            &studio_id, &item, &time_s, &collision_s, &angle_v, &velocity.x, &velocity.y, &angle, &position.x, &position.y);

    if (item > 0){
        item = 1<<item;
    }
}

void Robot::outputToString(char output[]){
    int target_studio = -1;
    if (target){
        target_studio = target->id;
    }
    sprintf(output, "Robot id: %d \nstudio_id: %d item: %d \ntime_s: %.3lf collision_s: %.3lf \nangle: %.3lf angle_v: %.3lf \nposition: (%.3lf, %.3lf) velocity: (%.3lf, %.3lf)\ntask: %d target: %d\n\n\n",
            id, studio_id, item, time_s, collision_s, angle, angle_v, position.x, position.y, velocity.x, velocity.y, task_now, target_studio);
}


void Robot::goToTargetStudio(){
    Point delta = target->position - position;
    double target_angle = atan2(delta.y, delta.x);
    double angle_delta = angleAdjust(target_angle - angle);
    setAngleV(angle_delta * 4);

    double v = 0;
    if (abs(angle_delta) < 1.5){
        v = 6;
        if (abs(delta) < 1){
            v = 3;
        }
    }

    velocity = Point(cos(angle), sin(angle))*v;
    printf("forward %d %lf\n", id, v);
}

void Robot::stop(){
    angle_v = 0;
    double v = 0;
    velocity = Point(sin(angle), cos(angle))*v;
    printf("rotate %d %lf\n", id, angle_v);
    printf("forward %d %lf\n", id, v);
}

void Robot::buy(){
    if (item != 0){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d try to sell buy while holding something! Item: %d\n", id, item);
#endif
        return;
    }
    if (studio_id == -1){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d try to buy without studio nearby!\n", id);
#endif
        return;
    }

    if (studio_list[studio_id].finish == 0){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d try to buy with studio working!\n", id);
#endif
        return;
    }

    printf("buy %d\n", id);
    item = PRODUCT[studio_list[studio_id].type];
    studio_list[studio_id].finish = 0;
    for (int i = 1; i <= 7; i++){
        if ((item>>i)&1){
            money -= COST[i];
        }
    }
}

void Robot::sell(){
    if (item == 0){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d try to sell without holding anything!\n", id);
#endif
        return;
    }
    if (studio_id == -1){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d try to sell without studio nearby!\n", id);
#endif
        return;
    }

    int item_sell = MATERIAL[studio_list[studio_id].type]&item;
    if (item_sell == 0 || (item_sell&studio_list[studio_id].item) != 0){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d try to sell with wrong studio! Studio id: %d, Item: %d, Studio item: %d\n", id, studio_id, item, studio_list[studio_id].item);
#endif
        return;
    }

    printf("sell %d\n", id);
    item ^= item_sell;
    studio_list[studio_id].item ^= item_sell;
    for (int i = 1; i <= 7; i++){
        if ((item_sell>>i)&1){
            money += VALUE[i];
        }
    }
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


void Robot::update(){
    if (task_now == Task::NONE){
        return;
    }
    if (task_now == Task::BUY && target->finish == 0){
        task_now = Task::NONE;
        target = NULL;
        return;
    }
    if (task_now == Task::BUY && studio_id == target->id){
        buy();
        task_now = Task::NONE;
        target = NULL;
        return;
    }
    if (task_now == Task::SELL && (target->item & item)){
        task_now = Task::NONE;
        target = NULL;
        return;
    }
    if (task_now == Task::SELL && studio_id == target->id){
        sell();
        task_now = Task::NONE;
        target= NULL;
        return;
    }
}

void Robot::dispatch(Task _task, Studio* _target){
    if (task_now != Task::NONE){
#ifdef DEBUG_MODE
        fprintf(warning_output, "[WARNING] Robot %d is 007 now!\n", id);
#endif
        return;
    }

    task_now = _task;
    target = _target;
    goToTargetStudio();
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

double Robot::getRadius(){
    return item? 0.53: 0.45;
}