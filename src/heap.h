#ifndef HEAP_H
#define HEAP_H
#include "main.h"
#include "game.h"

class GameInHeap{
public:
    Game* game;
    GameInHeap** another;
    double value;
    GameInHeap(Game* _game, double _value):game(_game), value(_value){
        another = NULL;
    };
};

class Heap{
public:
    GameInHeap** data[2];
    int size;
    int search_width;

    Heap(int _search_width);
    ~Heap();
    void heapSwap(int x, int y, int t);
    void upAdjust(int x, int t);
    void downAdjust(int x, int t);
    void pop(int x, int t);
    bool push(Game* g);
    Game* top();
};
#endif