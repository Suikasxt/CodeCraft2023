#include "heap.h"

Heap::Heap(int _search_width):size(0), search_width(_search_width){
    for (int i = 0; i < 2; i++){
        data[i] = new GameInHeap*[_search_width+2];
    }
};

void Heap::heapSwap(int x, int y, int t){
    swap(data[t][x], data[t][y]);
    (*(data[t][y]->another))->another = &(data[t][y]);
    (*(data[t][x]->another))->another = &(data[t][x]);
}

void Heap::upAdjust(int x, int t){
    if (x < 1 || x > size){
        return;
    }
    while (x > 1){
        int y = x>>1;
        if (data[t][x]->value < data[t][y]->value){
            heapSwap(x, y, t);
            x = y;
        }else{
            break;
        }
    }
}
void Heap::downAdjust(int x, int t){
    if (x < 1 || x > size){
        return;
    }
    while ((x<<1) <= size){
        int y = x<<1;
        if ((y|1) <= size && data[t][y|1]->value < data[t][y]->value){
            y |= 1;
        }
        if (data[t][y]->value < data[t][x]->value){
            heapSwap(x, y, t);
            x = y;
        }else{
            break;
        }
    }
}
void Heap::pop(int x, int t){
    int index_a = data[t][1]->another - data[t^1];

    heapSwap(1, size, t);
    heapSwap(index_a, size, t^1);
    delete data[t][size]->game;
    delete data[t][size];
    delete data[t^1][size];
    size--;
    upAdjust(index_a, t^1);
    downAdjust(index_a, t^1);
    downAdjust(1, t);
}
bool Heap::push(Game* g){
    g->calcValue();
    if (size >= search_width && g->value <= data[1][1]->value){
        return false;
    }
    size++;
    data[0][size] = new GameInHeap(g, g->frameID);
    data[1][size] = new GameInHeap(g, g->value);
    data[0][size]->another = &(data[1][size]);
    data[1][size]->another = &(data[0][size]);
    upAdjust(size, 0);
    upAdjust(size, 1);
    while (size > search_width){
        pop(1, 1);
    }
    return true;
}
Game* Heap::top(){
    return data[0][1]->game;
}

Heap::~Heap(){
    for (int i = 1; i <= size; i++){
        delete data[0][i]->game;
        delete data[0][i];
        delete data[1][i];
    }
    for (int i = 0; i < 2; i++){
        delete data[i];
    }
}