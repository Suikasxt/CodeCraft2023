#ifndef MAP_H
#define MAP_H
#include <utility>
#include "point.h"
#include "main.h"
#define MAP_SCALE 4
#define BLOCK_RANGE_LIMIT 4
#define BLOCK_RANGE_LIMIT_D BLOCK_RANGE_LIMIT*MAP_SCALE
using namespace std;

// orangesheee 修改
extern bool is_red;
extern bool block[50*MAP_SCALE][50*MAP_SCALE];
extern bool new_block[50*MAP_SCALE][50*MAP_SCALE];
extern bool block_nearby_4[50*MAP_SCALE+1][50*MAP_SCALE+1];
extern const double RADIUS[2];
extern pair<int, int> map_target[2][50][50*MAP_SCALE][50*MAP_SCALE];
extern pair<int, int> map_target_walkable[2][50][50*MAP_SCALE][50*MAP_SCALE];
extern double map_dist[2][50][50*MAP_SCALE][50*MAP_SCALE];
extern bool walk_available[2][50*MAP_SCALE][50*MAP_SCALE];
extern double studio_dist[50][50];
extern Point studio_dist_addition[50][50];
extern const int DIRECTION[8][2];

void readMap();
bool isBlock(int x, int y);
//pair<int, int> Continuous2Discrete(Point a);
pair<int, int> Continuous2DiscreteRound(Point a);
Point Discrete2Continuous(pair<int, int> a);
//Point Discrete2ContinuousCenter(pair<int, int> a);
bool DirectWalkWeak(int o, pair<int, int> a, pair<int, int> b);
bool DirectWalk(double radius, Point a, Point b);
double point2line_sqr(Point x, Point l_a, Point l_b);

#endif