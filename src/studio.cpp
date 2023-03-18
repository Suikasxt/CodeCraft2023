#include "studio.h"

Studio::Studio(int _id, int _type, Point _position)
:id(_id), type(_type), position(_position){
    item = 0;
    time_left = -1;
    finish = 0;
};

void Studio::readFromString(char input[]){
    sscanf(input, "%d %lf %lf %d %d %d",
            &type, &position.x, &position.y, &time_left, &item, &finish);
}

void Studio::outputToString(char output[]){
    sprintf(output, "Studio id: %d \ntype: %d \ntime_left: %d \nitem: %d \nfinish: %d \nposition: (%lf, %lf)\n",
            id, type, time_left, item, finish, position.x, position.y);
}
void Studio::update(){
    if (time_left == 0 && finish == 0){
        time_left = -1;
        if (type < 8){
            finish = 1;
        }
    }
    if (time_left == -1 && (item == MATERIAL[type] || (item && type == 9))){
        time_left = PERIOD[type];
        item = 0;
    }
}


const int MATERIAL[10] = {0, 0, 0, 0, 0x6, 0xa, 0xc, 0x70, 0x80, 0xfe};
const int PRODUCT[10] = {0, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0, 0};
const int PERIOD[10] = {0, 50, 50, 50, 500, 500, 500, 1000, 1, 1};
const int COST[8] = {0, 3000, 4400, 5800, 15400, 17200, 19200, 76000};
const int VALUE[8] = {0, 6000, 7600, 9200, 22500, 25000, 27500, 105000};