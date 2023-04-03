#include "map.h"
#include <stdio.h>
#include "main.h"
#include <cassert>
#include <queue>
#include <ctime>

const int MAP_SCALE = 4;

bool block[50*MAP_SCALE][50*MAP_SCALE];
bool block_corner[50*MAP_SCALE+1][50*MAP_SCALE+1];
bool walk_available[2][50*MAP_SCALE][50*MAP_SCALE];
const double RADIUS[2] = {0.45, 0.53};
const int DIRECTION[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
pair<int, int> map_target[2][50][50*MAP_SCALE][50*MAP_SCALE];
pair<int, int> map_target_walkable[2][50][50*MAP_SCALE][50*MAP_SCALE];
double map_dist[2][50][50*MAP_SCALE][50*MAP_SCALE];
pair<int, int> Continuous2Discrete(Point a){
    return make_pair(int(a.x*MAP_SCALE+EPS), int(a.y*MAP_SCALE+EPS));
}
pair<int, int> Continuous2DiscreteRound(Point a){
    return make_pair(int(a.x*MAP_SCALE+0.5), int(a.y*MAP_SCALE+0.5));
}
Point Discrete2Continuous(pair<int, int> a){
    return Point(a.first, a.second)*(1./MAP_SCALE);
}
Point Discrete2ContinuousCenter(pair<int, int> a){
    return Point(a.first+0.5, a.second+0.5)*(1./MAP_SCALE);
}
double pointToBlock(Point p, Point block){
    if (abs(p.x - block.x) < 0.25 && abs(p.y - block.y) < 0.25){
        return 0;
    }
    if (abs(p.x - block.x) < 0.25){
        return abs(p.y - block.y) - 0.25;
    }
    if (abs(p.y - block.y) < 0.25){
        return abs(p.x - block.x) - 0.25;
    }
    return abs(Point(abs(p.x - block.x) - 0.25, abs(p.y - block.y) - 0.25));
}
double pointToBlock(Point p, int x, int y){
    return pointToBlock(p, Discrete2Continuous(make_pair(x, y)));
}
bool isBlock(int x, int y){
    if (x < 0 || x >= 100){
        return true;
    }
    if (y < 0 || y >= 100){
        return true;
    }
    return block[x][y];
}
bool isBlock(pair<int, int> a){
    return isBlock(a.first, a.second);
}
double point2line_sqr(Point x, Point l_a, Point l_b){
    Point delta_x = x - l_a;
    Point delta_b = l_b - l_a;
    double inner_product = delta_x*delta_b;
    if (inner_product < 0){
        return abs2(delta_x);
    }
    double abs_delta_b = abs(delta_b);
    if (abs_delta_b < EPS){
        return abs2(delta_x);
    }
    double d = inner_product / abs_delta_b;
    if (d > abs_delta_b){
        return abs2(x - l_b);
    }
    return abs2(delta_x) - d*d;
}
bool DirectWalk(double radius, Point a, Point b){
    Point now = a;
    pair<int, int> last_pos(-1, -1);
    Point step = (b-a)*(0.2/abs(b-a));
    while (abs2(now-a) < abs2(b-a) + EPS){
        pair<int, int> pos = make_pair(int(now.x*2), int(now.y*2));
        if (pos != last_pos){
            last_pos = pos;
            
            if (isBlock(pos.first, pos.second)){
                return false;
            }
            for (int dx = 0; dx <= 1; dx++)
            for (int dy = 0; dy <= 1; dy++){
                int x = pos.first+dx;
                int y = pos.second+dy;
                if (block_corner[x][y]){
                    double min_dist = point2line_sqr(Point(x, y)*0.5, a, b);
                    if (min_dist < radius * radius){
                        return false;
                    }
                }
            }
        }
        now = now + step;
    }
    return true;
}
void roadSearch(int o, int studio_id){
    double r = RADIUS[o];
    for (int x = 0; x < 100; x++)
    for (int y = 0; y < 100; y++){
        map_dist[o][studio_id][x][y] = INF+1;
    }

    queue<pair<int, int>> q;
    pair<int, int> s_i_pos = Continuous2Discrete(studio_list[studio_id].position);
    int init_extern[12][2] = {{0, 0}, {0, 1}, {1, 0}, {1, 1}, {-1, 0}, {-1, 1}, {0, 2}, {1, 2}, {2, 1}, {2, 0}, {1, -1}, {0, -1}};
    for (int i = 0; i < 12; i++){
        int x = s_i_pos.first + init_extern[i][0];
        int y = s_i_pos.second + init_extern[i][1];
        map_dist[o][studio_id][x][y] = 0;
        map_target_walkable[o][studio_id][x][y] = s_i_pos;
        map_target[o][studio_id][x][y] = s_i_pos;
        if (isBlock(x, y) || walk_available[o][x][y] == false){
            continue;
        }
        q.push(make_pair(x, y));
    }
    while (!q.empty()){
        pair<int, int> now = q.front();
        pair<int, int> target_walkable = map_target_walkable[o][studio_id][now.first][now.second];
        q.pop();
        for (int i = 0; i < 4; i++){
            int x = now.first + DIRECTION[i][0];
            int y = now.second + DIRECTION[i][1];
            
            if (isBlock(x, y)){
                continue;
            }
            if (map_dist[o][studio_id][x][y] < INF){
                continue;
            }
            map_dist[o][studio_id][x][y] = map_dist[o][studio_id][now.first][now.second] + 0.5;
            if (DirectWalk(r, Point(x, y)*0.5, Point(target_walkable.first, target_walkable.second)*0.5)){
                map_target_walkable[o][studio_id][x][y] = target_walkable;
            }else{
                map_target_walkable[o][studio_id][x][y] = now;
            }

            if (walk_available[o][x][y] == false){
                continue;
            }
            q.push(make_pair(x, y));
        }
    }

}
void mapInit(){
    for (int x = 0; x <= 100; x++)
    for (int y = 0; y <= 100; y++){
        block_corner[x][y] = isBlock(x, y) || isBlock(x-1, y) || isBlock(x, y-1) || isBlock(x-1, y-1);
    }
    for (int o = 0; o < 2; o++){
        double r = RADIUS[o];
        for (int x = 0; x < 100; x++)
        for (int y = 0; y < 100; y++){
            walk_available[o][x][y] = true;
            Point position = Discrete2Continuous(make_pair(x, y));
            for (int dx = -3; dx <= 3; dx++)
            for (int dy = -3; dy <= 3; dy++)
            if (isBlock(x+dx, y+dy)){
                if (pointToBlock(position, x+dx, y+dx) < r){
                    walk_available[o][x][y] = false;
                }
            }
        }
        for (auto studio = studio_list.begin(); studio != studio_list.end(); studio++){
            roadSearch(o, studio->id);
        }
    }

#ifdef DEBUG
    for (int i = 99; i >= 0; i--){
        for (int j = 0; j < 100; j++){
            fprintf(warning_output, "%d", walk_available[1][j][i]);
        }
        fprintf(warning_output, "\n");
        fflush(warning_output);
    }
#endif
}
void readMap(){
    int t_start = clock();
    for (int i = 0; i < 10; i++){
        studio_dict[i].clear();
    }
    studio_list.clear();
    robot_list.clear();
    
    for (int i = 99; i >= 0; i--){
        scanf("%s", map[i]);
        for (int j = 0; j < 100; j++){
            if (map[i][j] == 'A'){
                int id = robot_list.size();
                robot_list.push_back(Robot(id, Point(j+0.5, i+0.5)*0.5));
            }
            else if (map[i][j] >= '1' && map[i][j] <= '9'){
                int id = studio_list.size();
                studio_list.push_back(Studio(id, map[i][j]-'0', Point(j+0.5, i+0.5)*0.5));
            }
            else if (map[i][j] == '#'){
                block[j][i] = true;
            }
        }
    }
    for (int i = 0; i < studio_list.size(); i++){
        studio_dict[studio_list[i].type].push_back(&(studio_list[i]));
    }
    char line[10];
    scanf("%s", line);
    assert(line[0] == 'O' && line[1] == 'K');
    fprintf(stderr, "Map init start\n");
    mapInit();
    int t_end = clock();
    fprintf(stderr, "%lf\n", 1.*(t_end - t_start)/CLOCKS_PER_SEC);
    //assert(1.*(t_end - t_start)/CLOCKS_PER_SEC < 5);
}