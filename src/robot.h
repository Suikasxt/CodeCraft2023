#ifndef ROBOT_H
#define ROBOT_H

#include "point.h"
#include "studio.h"
#include <vector>
using namespace std;

enum Task{
    NONE, BUY, SELL, STOP, WAIT
};

class Robot{
public:
    int id;
    int studio_id;
    int item;
    int data_frameID;
    int pick_up_time;
    int target;
    Task task_now;
    double time_s;
    double collision_s;
    double angle;
    double angle_v;
    int last_target;
    Point position;
    Point velocity;

    Robot(int _id, Point _position);
    void readFromString(char input[]);
    void outputToString(char output[]);
    void stop();
    int buy(vector<Studio>& studio_list, int frameID, bool output = false);
    int sell(vector<Studio>& studio_list, int frameID, bool output = false);
    void destroy();
    void goToTargetStudio(Studio* studio);
    
    int update(vector<Studio>& studio_list, int frameID, bool output = false);
    void dispatch(Studio* studio, bool output = false);

    void setAngleV(double _angle_v);
    void setVelocity(double _angle_v);
    double getRadius();
    void flushTimeS(int frameID);
};
#endif