#ifndef MAIN_H
#define MAIN_H

#define _USE_MATH_DEFINES
#include <vector>
#include "robot.h"
#include "studio.h"
#include "game.h"
using namespace std;

#ifdef _LOCAL
#define DEBUG_MODE
#endif
#define FRAME_PRE_SEC 50
#define INF 1e9
#define EPS 1e-5

extern vector<Studio> studio_list;
extern vector<Robot> robot_list;
extern FILE* warning_output;
extern int money;
extern int frameID;
extern char map[110][110];
extern int map_num;


#pragma GCC optimize("Ofast")

#endif