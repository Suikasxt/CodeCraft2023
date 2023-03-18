#ifndef STUDIO_H
#define STUDIO_H

#include "point.h"
#include <stdio.h>

class Studio{
public:
    int id;
    int type;
    Point position;
    int time_left;
    int item;
    bool finish;
    Studio(int _id, int _type, Point _position);
    void readFromString(char input[]);
    void outputToString(char output[]);
    void update();
};

extern const int MATERIAL[10];
extern const int PRODUCT[10];
extern const int PERIOD[10];
extern const int COST[8];
extern const int VALUE[8];
#endif