#include "map.h"
#include <stdio.h>
#include "main.h"
#include <cassert>
#include <queue>
#include <ctime>

// orangesheee 修改，增加了对决赛不同地图的支持，判断红蓝方
bool is_red = false;
char robot_syms[] ={'A','B'};
char studio_syms[] = {'1','a'};

bool debug_output = false;
bool block[50*MAP_SCALE][50*MAP_SCALE];
bool new_block[50*MAP_SCALE][50*MAP_SCALE];
bool block_nearby_4[50*MAP_SCALE+1][50*MAP_SCALE+1];
bool block_nearby_4_4[50*MAP_SCALE+1][50*MAP_SCALE+1];
double nearest_block[200][200];
bool walk_available[2][50*MAP_SCALE][50*MAP_SCALE];
double studio_dist[50][50];
Point studio_dist_addition[50][50];
const double RADIUS[2] = {0.45, 0.53};
const int DIRECTION[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1}};
pair<int, int> map_target[2][50][50*MAP_SCALE][50*MAP_SCALE];
double map_dist[2][50][50*MAP_SCALE][50*MAP_SCALE];
/*pair<int, int> Continuous2Discrete(Point a){
    return make_pair(int(a.x*MAP_SCALE+EPS), int(a.y*MAP_SCALE+EPS));
}*/
pair<int, int> Continuous2DiscreteRound(Point a){
    return make_pair(int(a.x*MAP_SCALE+0.5), int(a.y*MAP_SCALE+0.5));
}
Point Discrete2Continuous(pair<int, int> a){
    return Point(a.first, a.second)*(1./MAP_SCALE);
}
/*Point Discrete2ContinuousCenter(pair<int, int> a){
    return Point(a.first+0.5, a.second+0.5)*(1./MAP_SCALE);
}*/
int gcd(int x, int y){
    return y?gcd(y, x%y):x;
}
/*
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
*/
bool isBlock(int x, int y){
    if (x < 0 || x >= 50*MAP_SCALE){
        return true;
    }
    if (y < 0 || y >= 50*MAP_SCALE){
        return true;
    }
    return block[x][y] || new_block[x][y];
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
    Point step = (b-a)*(0.05/abs(b-a));
    while (abs2(now-a) < abs2(b-a) + EPS){
        pair<int, int> pos(Continuous2DiscreteRound(now));
        if (pos != last_pos){
            last_pos = pos;
            
            if (isBlock(pos.first, pos.second)){
                return false;
            }
            for (int dx = -2; dx <= 2; dx++)
            for (int dy = -2; dy <= 2; dy++){
                int x = pos.first+dx;
                int y = pos.second+dy;
                if (isBlock(x, y)){
                    if (point2line_sqr(Discrete2Continuous(make_pair(x, y)), a, b) < radius * radius){
                        return false;
                    }
                }
            }
        }
        now = now + step;
    }
    return true;
}
bool DirectWalkWeak(int o, pair<int, int> a, pair<int, int> b){
    //return false;
    pair<int, int> delta(b.first - a.first, b.second - a.second);
    pair<int, int> f(delta.first >= 0? 1:-1, delta.second >= 0? 1:-1);
    int g = gcd(abs(delta.first),abs(delta.second));
    pair<int, int> delta_part(delta.first/g, delta.second/g);
    for (int i = 0; i < g; i++){
        if (!walk_available[o][a.first][a.second]){
            return false;
        }
        int last = 0;
        if (abs(delta_part.first) > abs(delta_part.second)){
            double rate = 1.* delta_part.second / delta_part.first;
            for (int _x = 1; _x < abs(delta_part.first); _x++){
                int x = _x*f.first - (f.first==1);
                int y = floor(rate*x);
                if (block_nearby_4_4[a.first+x][a.second+y]){
                    return false;
                }
                if (last != y){
                    if (block_nearby_4_4[a.first+x][a.second+last]){
                        return false;
                    }
                    last = y;
                }
            }
        }else{
            double rate = 1.* delta_part.first / delta_part.second;
            for (int _y = 1; _y < abs(delta_part.second); _y++){
                int y = _y*f.second - (f.second==1);
                int x = floor(rate*y);
                if (block_nearby_4_4[a.first+x][a.second+y]){
                    return false;
                }
                if (last != x){
                    if (block_nearby_4_4[a.first+last][a.second+y]){
                        return false;
                    }
                    last = x;
                }
            }
        }
        a.first += delta_part.first;
        a.second += delta_part.second;
    }
    if (!walk_available[o][b.first][b.second]){
        return false;
    }
    return true;
}
void roadSearch(int o, int studio_id){
    double r = RADIUS[o];
    for (int x = 0; x < 50*MAP_SCALE; x++)
    for (int y = 0; y < 50*MAP_SCALE; y++){
        map_dist[o][studio_id][x][y] = INF+1;
    }

    priority_queue<pair<double, pair<int, int>>> q;
    pair<int, int> s_i_pos = Continuous2DiscreteRound(studio_list[studio_id].position);
    for (int dx = -2; dx <= 2; dx++)
    for (int dy = -2; dy <= 2; dy++){
        if (abs(Point(dx, dy))*(1./MAP_SCALE) > 0.4){
            continue;
        }
        int x = s_i_pos.first + dx;
        int y = s_i_pos.second + dy;
        map_dist[o][studio_id][x][y] = 0;
        map_target[o][studio_id][x][y] = s_i_pos;
        if (isBlock(x, y) || walk_available[o][x][y] == false){
            continue;
        }
        q.push(make_pair(-map_dist[o][studio_id][x][y], make_pair(x, y)));
    }
    double direction_dist[8];
    for (int i = 0; i < 8; i++){
        direction_dist[i] = abs(Point(DIRECTION[i][0], DIRECTION[i][1]))*(1./MAP_SCALE);
    }
    while (!q.empty()){
        pair<int, int> now = q.top().second;
        q.pop();
        for (int i = 0; i < 8; i++){
            int x = now.first + DIRECTION[i][0];
            int y = now.second + DIRECTION[i][1];
            pair<int, int> next = make_pair(x, y);
            
            if (isBlock(x, y)){
                continue;
            }
            double dist_now = map_dist[o][studio_id][now.first][now.second] + direction_dist[i] + (BLOCK_RANGE_LIMIT - nearest_block[x][y])*0.1;
            if (map_dist[o][studio_id][x][y] <= dist_now){
                continue;
            }
            map_dist[o][studio_id][x][y] = dist_now;
            map_target[o][studio_id][x][y] = now;
            if (walk_available[o][x][y] == false){
                continue;
            }
            q.push(make_pair(-map_dist[o][studio_id][x][y], next));
        }
    }

}
void mapInit(){
    double pre_calc_dist[BLOCK_RANGE_LIMIT_D*2+1][BLOCK_RANGE_LIMIT_D*2+1];
    
    for (int dx = -BLOCK_RANGE_LIMIT_D; dx < BLOCK_RANGE_LIMIT_D; dx++)
    for (int dy = -BLOCK_RANGE_LIMIT_D; dy < BLOCK_RANGE_LIMIT_D; dy++){
        pre_calc_dist[dx+BLOCK_RANGE_LIMIT_D][dy+BLOCK_RANGE_LIMIT_D] = abs(Discrete2Continuous(make_pair(dx, dy)));
    }
    for (int x = 0; x < 50*MAP_SCALE; x++)
    for (int y = 0; y < 50*MAP_SCALE; y++){
        nearest_block[x][y] = BLOCK_RANGE_LIMIT;
        for (int dx = -BLOCK_RANGE_LIMIT_D; dx < BLOCK_RANGE_LIMIT_D; dx++)
        for (int dy = -BLOCK_RANGE_LIMIT_D; dy < BLOCK_RANGE_LIMIT_D; dy++)
        if (isBlock(x+dx, y+dy) && nearest_block[x][y] > pre_calc_dist[dx+BLOCK_RANGE_LIMIT_D][dy+BLOCK_RANGE_LIMIT_D]){
            nearest_block[x][y] = pre_calc_dist[dx+BLOCK_RANGE_LIMIT_D][dy+BLOCK_RANGE_LIMIT_D];
        }
    }
    for (int x = 0; x <= 50*MAP_SCALE; x++)
    for (int y = 0; y <= 50*MAP_SCALE; y++){
        block_nearby_4[x][y] = isBlock(x, y);
        for (int i = 0; i < 4; i++){
            block_nearby_4[x][y] |= isBlock(x+DIRECTION[i][0], y+DIRECTION[i][1]);
        }
    }
    for (int x = 0; x < 50*MAP_SCALE; x++)
    for (int y = 0; y < 50*MAP_SCALE; y++){
        block_nearby_4_4[x][y] = false;
        for (int dx = 0; dx <= 1; dx++)
        for (int dy = 0; dy <= 1; dy++){
            block_nearby_4_4[x][y] |= block_nearby_4[x+dx][y+dy];
        }
    }
    for (int o = 0; o < 2; o++){
        double r = RADIUS[o];
        for (int x = 0; x < 50*MAP_SCALE; x++)
        for (int y = 0; y < 50*MAP_SCALE; y++){
            walk_available[o][x][y] = true;
            Point position = Discrete2Continuous(make_pair(x, y));
            for (int dx = -3; dx <= 3; dx++)
            for (int dy = -3; dy <= 3; dy++)
            if (isBlock(x+dx, y+dy)){
                if (abs(Point(dx, dy))*(1./MAP_SCALE) < r){
                    walk_available[o][x][y] = false;
                }
            }
        }
        for (auto studio = studio_list.begin(); studio != studio_list.end(); studio++){
            roadSearch(o, studio->id);
            for (auto studio_B = studio_list.begin(); studio_B != studio_list.end(); studio_B++){
                pair<int, int> coord = Continuous2DiscreteRound(studio_B->position);
                studio_dist[studio->id][studio_B->id] = map_dist[1][studio->id][coord.first][coord.second]; 
            }
        }
    }
    
    for (auto studio = studio_list.begin(); studio != studio_list.end(); studio++)
    for (auto next_studio = studio_list.begin(); next_studio != studio_list.end(); next_studio++){
        pair<int, int> coord_studio = Continuous2DiscreteRound(studio->position);
        studio_dist_addition[studio->id][next_studio->id] = studio->position;
        studio_dist[studio->id][next_studio->id] = map_dist[1][next_studio->id][coord_studio.first][coord_studio.second];
        for (int dx = -2; dx <= 2; dx++)
        for (int dy = -2; dy <= 2; dy++){
            if (map_dist[1][next_studio->id][coord_studio.first + dx][coord_studio.second + dy] < studio_dist[studio->id][next_studio->id] &&
                map_dist[1][next_studio->id][coord_studio.first + dx][coord_studio.second + dy] < map_dist[1][next_studio->id][coord_studio.first][coord_studio.second] - 3){
                studio_dist[studio->id][next_studio->id] = map_dist[1][next_studio->id][coord_studio.first + dx][coord_studio.second + dy];
                studio_dist_addition[studio->id][next_studio->id] = Discrete2Continuous(make_pair(coord_studio.first + dx, coord_studio.second + dy));
            }
        }
        if (abs(studio_dist_addition[studio->id][next_studio->id] - studio->position) > 0.385){
            Point step = studio_dist_addition[studio->id][next_studio->id] - studio->position;
            step = step * (1./abs(step));
            studio_dist_addition[studio->id][next_studio->id] = studio->position + step*0.385;
        }
    }

#ifdef DEBUG_MODE
    /*for (int i = 50*MAP_SCALE-1; i >= 0; i--){
        for (int j = 0; j < 50*MAP_SCALE; j++){
            fprintf(warning_output, "%d", block_nearby_4[j][i]);
        }
        fprintf(warning_output, "\n");
        fflush(warning_output);
    }*/
#endif
}
void readMap(){
    int t_start = clock();
    studio_list.clear();
    robot_list.clear();

    for (int i = 99; i >= 0; i--){
        scanf("%s", map[i]);
        for (int j = 0; j < 100; j++){
            if (map[i][j] == robot_syms[int(is_red)]){
                int id = robot_list.size();
                robot_list.push_back(Robot(id, Point(j+0.5, i+0.5)*0.5));
            }
            else if (map[i][j] >= studio_syms[int(is_red)] && map[i][j] <= studio_syms[int(is_red)]+8){
                int id = studio_list.size();
                studio_list.push_back(Studio(id, map[i][j]-studio_syms[int(is_red)]+1, Point(j+0.5, i+0.5)*0.5));
            }
            else if (map[i][j] == '#'){
                for (int dx = 0; dx <= 2; dx++)
                for (int dy = 0; dy <= 2; dy++)
                block[j*2+dx][i*2+dy] = true;
            }
        }
    }
    char line[10];
    scanf("%s", line);
    assert(line[0] == 'O' && line[1] == 'K');
    fprintf(stderr, "Map init start\n");
    mapInit();
    int t_end = clock();
    fprintf(stderr, "%lf\n", 1.*(t_end - t_start)/CLOCKS_PER_SEC);
    //debug_output = true;
    //assert(1.*(t_end - t_start)/CLOCKS_PER_SEC < 5);
}