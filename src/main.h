#ifndef MAIN_H
#define MAIN_H

#include <vector>
#include "robot.h"
#include "studio.h"
using namespace std;

//#define DEBUG_MODE
#define FRAME_PRE_SEC 50
#define INF 1e9
#define EPS 1e-9

extern vector<Studio> studio_list;
extern vector<Robot> robot_list;
extern vector<Studio*> studio_dict[10];
extern FILE* warning_output;
extern int money;
extern int frameID;
extern char map[110][110];

#endif