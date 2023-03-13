#include "point.h"
#include <cmath>

Point operator + (Point a, Point b){
    return Point(a.x+b.x, a.y+b.y);
}
Point operator - (Point a, Point b){
    return Point(a.x-b.x, a.y-b.y);
}
Point operator * (Point a, double s){
    return Point(a.x*s, a.y*s);
}
double abs2(Point a){
    return a.x*a.x + a.y*a.y;
}
double abs(Point a){
    return sqrt(abs2(a));
}

double angleAdjust(double angle){
    if (angle > M_PI){
        angle -= M_PI*2;
    }
    if (angle < -M_PI){
        angle += M_PI*2;
    }
    return angle;
}