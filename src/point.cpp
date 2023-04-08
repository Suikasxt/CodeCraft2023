#include "point.h"
#include <math.h>

Point operator + (Point a, Point b){
    return Point(a.x+b.x, a.y+b.y);
}
Point operator - (Point a, Point b){
    return Point(a.x-b.x, a.y-b.y);
}
Point operator * (Point a, double s){
    return Point(a.x*s, a.y*s);
}

double operator * (Point a, Point b){
    return a.x*b.x + a.y*b.y;
}
double abs2(Point a){
    return sqr(a.x) + sqr(a.y);
}
double abs(Point a){
    return sqrt(abs2(a));
}

double absManhattan(Point a){
    return fabs(a.x) + fabs(a.y);
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
double sqr(double x){
    return x*x;
}