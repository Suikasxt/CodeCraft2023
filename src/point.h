#define _USE_MATH_DEFINES
#ifndef POINT_H
#define POINT_H
#include <cmath>

struct Point{
    double x,y;
    Point(double _x=0, double _y=0):x(_x),y(_y){};
};

Point operator + (Point a, Point b);
Point operator - (Point a, Point b);
Point operator * (Point a, double s);
double abs2(Point a);
double abs(Point a);
double angleAdjust(double angle);
double absManhattan(Point a);
#endif