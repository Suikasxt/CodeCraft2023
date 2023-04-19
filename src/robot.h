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
    int next_target;
    int target_action_num;
    Task task_now;
    double time_s;
    double target_angle_future;
    double collision_s;
    double angle;
    double angle_v;
    double original_angle_v;
    double position_v;
    Point position;
    Point velocity;
    Point original_velocity;
    double radar[360];

    Point additional_target_position;

    int last_target;

    double additional_angle_v;
    double additional_pos_v;
    int additional_frame_num;

    Robot(int _id, Point _position);
    void readFromString(char input[]);
    void outputToString(char output[]);
    void stop();
    int buy(vector<Studio>& studio_list, int frameID, bool output = false);
    int sell(vector<Studio>& studio_list, int frameID, bool output = false);
    void destroy();
    
    void goToTargetPosition(Point target, bool output=false);
    void goToTargetStudio(Studio* studio, bool output=false);
    void goToTargetPath(vector<pair<int, int>> &path, bool output=false);
    
    int update(vector<Studio>& studio_list, int frameID, bool output = false);
    void dispatch(Studio* studio, int action_num, bool output = false);

    void setAngleV(double _angle_v, bool output=false);
    void setVelocity(double _angle_v, bool output=false);
    double getRadius();
    void flushTimeS(int frameID);
    void physicalUpdate(double time);
};
#endif