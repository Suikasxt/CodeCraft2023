#ifndef ROBOT_H
#define ROBOT_H

#include "point.h"
#include "studio.h"

enum Task{
    NONE, BUY, SELL
};

class Robot{
public:
    int id;
    int studio_id;
    int item;
    Studio *target;
    Task task_now;
    double time_s;
    double collision_s;
    double angle;
    double angle_v;
    Point position;
    Point velocity;

    Robot(int _id, Point _position);
    void readFromString(char input[]);
    void outputToString(char output[]);
    void stop();
    void buy();
    void sell();
    void destroy();
    void goToTargetStudio();
    
    void update();
    void dispatch(Task _task, Studio* _target);

    void setAngleV(double _angle_v);
    double getRadius();
};
#endif