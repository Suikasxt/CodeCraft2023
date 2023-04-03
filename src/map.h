#ifndef MAP_H
#define MAP_H
#include <utility>
#include "point.h"
#include "main.h"
using namespace std;

extern bool block[100][100];
extern const double RADIUS[2];
void readMap();
extern pair<int, int> map_target_walkable[2][50][100][100];
extern double map_dist[2][50][100][100];

pair<int, int> Continuous2Discrete(Point a);
pair<int, int> Continuous2DiscreteRound(Point a);
Point Discrete2Continuous(pair<int, int> a);
Point Discrete2ContinuousCenter(pair<int, int> a);

#endif