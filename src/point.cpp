#include "point.h"

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