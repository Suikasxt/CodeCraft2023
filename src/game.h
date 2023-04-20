#ifndef GAME_H
#define GAME_H

#define _USE_MATH_DEFINES
#include <vector>
#include <stack>
#include "main.h"
#include "studio.h"
#include "robot.h"
using namespace std;

struct UpdateRoad{
    int pre_id;
    int robot_id;
    int target_id;
    int action_num;
    UpdateRoad(int _pre_id, int _robot_id, int _target_id, int _action_num):pre_id(_pre_id), robot_id(_robot_id), target_id(_target_id), action_num(_action_num){};
};

int moveTimePredict(Robot* robot);
class Game{
public:
    vector<Studio> studio_list;
    vector<Robot> robot_list;
    double value;
    int frameID;
    int money;
    int road_id;


    Game(vector<Studio> &_studio_list, vector<Robot> &_robot_list, int _frameID, int _money);
    void greedyWork(double value_list[4][50] = NULL);
};

#endif