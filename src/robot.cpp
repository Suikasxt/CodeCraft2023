#include "robot.h"
#include "main.h"
#include <stdio.h>
#include <math.h>
#include "map.h"

const double ROBOT_DENSITY = 20;
const double ROBOT_FORCE = 250;
const double ROBOT_MOMENT = 50;

Robot::Robot(int _id, Point _position)
:id(_id),position(_position),task_now(NONE),target(-1),last_target(-1){
    data_frameID = 0;
    angle = 0;
    item = 0;
    angle_v = 0;
    time_s = 0;
    collision_s = 0;
    original_angle_v = 0;
    additional_frame_num = 0;
    target_action_num = -1;
};

void Robot::readFromString(char input[])
{
    sscanf(input, "%d %d %lf %lf %lf %lf %lf %lf %lf %lf",
            &studio_id, &item, &time_s, &collision_s, &angle_v, &velocity.x, &velocity.y, &angle, &position.x, &position.y);

    if (item > 0){
        item = 1<<item;
    }
    data_frameID = frameID;
    original_angle_v = angle_v;
    original_velocity = velocity;
}

void Robot::outputToString(char output[]){
    sprintf(output, "Robot id: %d \nstudio_id: %d item: %d \ntime_s: %.3lf collision_s: %.3lf \nangle: %.6lf angle_v: %.6lf \nposition: (%.6lf, %.6lf) velocity: (%.6lf, %.6lf)\ntask: %d target: %d\n\n\n",
            id, studio_id, item, time_s, collision_s, angle, original_angle_v, position.x, position.y, velocity.x, velocity.y, task_now, target);
}

void Robot::goToTargetPosition(Point target, bool output){
    Point delta = target - position;
    double target_angle = atan2(delta.y, delta.x);
    double angle_delta = angleAdjust(target_angle - angle - original_angle_v/FRAME_PRE_SEC);

    bool reverse = false;
    if (abs(delta) < 2 && abs(angle_delta) > 2.5){
    //    angle_delta = angleAdjust(angle_delta + M_PI);
    //    reverse = true;
    }

    double r = getRadius();
    double mass = M_PI * r * r * ROBOT_DENSITY;
    double inertia = mass * r * r / 2;
    double angle_v_a = ROBOT_MOMENT / inertia;
    double angle_v_a_one_frame = angle_v_a/FRAME_PRE_SEC;
    if (fabs(angle_delta) < 0.0001 && fabs(original_angle_v) < angle_v_a_one_frame){
        setAngleV(angle_delta*FRAME_PRE_SEC, output);
    }else if (fabs(angle_delta)*FRAME_PRE_SEC < angle_v_a_one_frame && fabs(angle_delta*FRAME_PRE_SEC - original_angle_v) < angle_v_a_one_frame){
        setAngleV(angle_delta*FRAME_PRE_SEC, output);
    }else{
        int step = int(fabs(original_angle_v)/angle_v_a_one_frame);
        double angle_delta_after_stop = angle_delta - (original_angle_v/FRAME_PRE_SEC*step - step*step/2*angle_v_a_one_frame/FRAME_PRE_SEC);
        if (fabs(angle_delta_after_stop)*FRAME_PRE_SEC < angle_v_a_one_frame){
            setAngleV(0, output);
        }else{
            setAngleV(angle_delta_after_stop > 0 ? M_PI:-M_PI, output); 
        }
        //fprintf(warning_output, "%lf %lf %lf %lf\n", original_angle_v, angle_delta_after_stop, angle_delta, angle_v);
    }// To be upgrade
    //setAngleV(angle_delta*3, output);
    
    double v = 0;
    if (fabs(angle_delta) < 0.4){
        v = 6;
        if (item){
            v = fmin(v, abs(delta) * 5);
        }else{
            v = fmin(v, abs(delta) * 12);
        }
        /*if (studio->action_num < target_action_num){
            if (abs(delta) < 0.2){
                v = 0;
            }
        }*/
        if (reverse){
            v = -2;
        }
    }

    setVelocity(v, output);

#ifdef DEBUG_MODE
    if (output)
    fprintf(warning_output, "%d pos: %lf %lf, angle:%lf %lf  (%lf, %lf)  (%lf, %lf)\n", id, abs(delta), position_v, angle_delta, angle_v, position.x, position.y, target.x, target.y);
#endif
}

void Robot::goToTargetPath(vector<pair<int, int>> &path, bool output){
    int j = 0;
    for (int i = 20; i>=0; i--){
        if (j + (1<<i) < path.size() && DirectWalk(RADIUS[item>0], position, Discrete2Continuous(path[j+(1<<i)]))){
            j += 1<<i;
        }
    }
    pair<int, int> target_position_coord = path[j];
    
    Point target_position = Discrete2Continuous(target_position_coord);
    goToTargetPosition(target_position, output);
}

void Robot::goToTargetStudio(Studio* studio, bool output){
    pair<int, int> coord = Continuous2DiscreteRound(position);
    
    pair<int, int> target = map_target[item>0][studio->id][coord.first][coord.second];
    vector<pair<int, int>> path;
    path.push_back(target);
    while(target != map_target[item>0][studio->id][target.first][target.second]){
        target = map_target[item>0][studio->id][target.first][target.second];
        path.push_back(target);
    }
    if (output){
#ifdef DEBUG_MODE
        pair<int, int> coord = Continuous2DiscreteRound(position);
        Point now_d = Discrete2Continuous(coord);
        fprintf(warning_output, "%d %d now: (%lf, %lf) dist: %lf\n", id, studio->id, now_d.x, now_d.y, map_dist[item>0][studio->id][coord.first][coord.second]);
#endif
    }
    goToTargetPath(path, output);
}

void Robot::stop(){
    setVelocity(0);
    setAngleV(0);
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
    studio_list[studio_id].action_num++;
    studio_list[studio_id].update();
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
    studio_list[studio_id].action_num++;
    studio_list[studio_id].update();
    
    int item_id = 0;
    for (int x=(item_sell>>1); x; x>>=1) item_id++;
    flushTimeS(frameID);
    money += int(VALUE[item_id] * time_s * collision_s);
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
    if (target_studio->action_num < target_action_num){
        return money;
    }
    if (task_now == Task::BUY && target_studio->finish == 0){
        return money;
    }
    //fprintf(warning_output, "%d r_pos: (%lf %lf) s_pos:(%lf %lf) t:(%lf %lf)\n", id, position.x, position.y, target_studio->position.x, target_studio->position.y, additional_target_position.x, additional_target_position.y);
    if (task_now == Task::BUY && studio_id == target_studio->id && (abs(position - additional_target_position) < 0.015 || abs(target_studio->position - additional_target_position) < 0.1)){
        money += buy(studio_list, frameID, output);
        if (money){
            task_now = Task::NONE;
            last_target = target;
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
            last_target = target;
            target = -1;
        }
        return money;
    }
    return money;
}


void Robot::physicalUpdate(double time){
    double r = getRadius();
    double mass = M_PI * r * r * ROBOT_DENSITY;
    double inertia = mass * r * r / 2;
    double angle_v_a = ROBOT_MOMENT / inertia;
    double velocity_a = ROBOT_FORCE / mass;

    double original_v = abs(original_velocity);
    double v = abs(velocity);
    if (abs(v - original_v) > velocity_a * time){
        if (v < original_v){
            v = original_v - velocity_a * time;
        }else{
            v = original_v + velocity_a * time;
        }
    }
    velocity = Point(cos(angle), sin(angle)) * v;


    if (abs(angle_v - original_angle_v) > angle_v_a * time){
        if (angle_v < original_angle_v){
            angle_v = original_angle_v - angle_v_a * time;
        }else{
            angle_v = original_angle_v + angle_v_a * time;
        }
    }


    position = position + original_velocity*time;
    angle = angleAdjust(angle + original_angle_v*time);

    
    original_velocity = velocity;
    original_angle_v = angle_v;
}

void Robot::dispatch(Studio* studio, int action_num, bool output){
    if (item){
        task_now = Task::SELL;
    }else{
        task_now = Task::BUY;
    }
    target = studio->id;
    target_action_num = action_num;
    goToTargetStudio(studio, output);
}


void Robot::setAngleV(double _angle_v, bool output){
    angle_v = _angle_v;
    if (angle_v > M_PI - EPS){
        angle_v = M_PI - EPS;
    }
    if (angle_v < -M_PI + EPS){
        angle_v = -M_PI + EPS;
    }
    //fprintf(stderr, "%d %lf\n", id, angle_v);
    if (output){
        printf("rotate %d %.10lf\n", id, angle_v);
    }
}
void Robot::setVelocity(double v, bool output){
    if (v >= 6){
        v = 6;
    }
    if (v <= -2){
        v = -2;
    }
    position_v = v;
    velocity = Point(cos(angle), sin(angle))*v;
    if (output){
        printf("forward %d %lf\n", id, v);
    }
}

double Robot::getRadius(){
    return RADIUS[item>0];
}


void Robot::flushTimeS(int frameID){
    double rate = 1 - 1.*(frameID - pick_up_time) / 9000;
    time_s = 0.8 + 0.2*(1-sqrt(1-rate*rate));
}