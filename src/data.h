

#define WORK_LIST_LENGTH 200
#define PRE_WORK_MAP_NUM 4
int PRE_WORK_DATA[PRE_WORK_MAP_NUM][4][WORK_LIST_LENGTH][2] = {
    {
        {41, 19, 41, 36, 42, 32, 42, 38, 42, 13, 41, 31, 42, 31, 31, 15, 41, 31, 42, 31, 31, 10, 41, 31, 41, 31, 31, 30, 41, 39, 39, 30, 41, 31, 42, 38, 38, 30, 41, 38, 38, 30, -1},
        {42, 40, 42, 33, 41, 35, 41, 28, 41, 31, 42, 31, 31, 22, 41, 31, 42, 31, 41, 38, 42, 38, 41, 38, 42, 31, 42, 39, 42, 39, 39, 30, 41, 31, 42, 8, -1},
        {41, 26, 41, 27, 41, 20, 41, 37, 41, 38, 42, 38, 38, 23, 41, 38, 42, 38, 42, 38, 38, 11, 41, 38, 38, 18, 42, 38, 38, 30, 41, 38, 42, 38, 31, 18, 42, 31, 42, 31, 31, 30, 42, 31, 31, 30, 38, -1},
        {42, 34, 42, 25, 42, 39, 42, 24, 42, 31, 41, 38, 42, 38, 38, 21, 41, 38, 38, 12, 31, 17, 41, 31, 42, 31, 31, 30, 41, 39, 42, 38, 38, 30, 41, 38, 38, 30, 41, 38, 42, 38, 42, 38, -1},
    }, // MAP 1 local 534149
    {
        {6, 12, 8, 1, 4, 1, 4, 12, 12, 21, 20, 15, 13, 1, 4, 1, 4, 15, 17, 24, 17, 15, 8, 1, 6, 0, 7, 0, 0, 3, 3, 14, 17, 23, 18, 22, 22, 21, 20, 12, 8, 2, 2, 3, 4, 23, 23, 21, 21, 14, 8, 1, 4, 12, 17, 24, 15, 3, 6, 2, -1},
        {7, 9, 11, 12, 18, 12, 5, 0, 11, 9, 7, 1, 1, 3, 4, 12, 8, 1, 4, 23, 20, 9, 16, 23, 18, 24, 17, 2, 8, 1, 6, 2, 6, 0, 5, 12, 12, 21, 19, 12, 6, 2, 2, 3, 3, 10, 11, 1, 6, 0, -1},
        {8, 15, 13, 15, 13, 12, 6, 12, 19, 12, 12, 3, 4, 12, 7, 9, 11, 9, 7, 0, 7, 9, 7, 0, 0, 3, 4, 1, 1, 3, 5, 0, 7, 22, 21, 14, 13, 1, 12, 21, 18, 22, 23, 21, 20, 12, 12, 3, 8, 2, 1, 3, 5, 12, 12, 21, 20, 12, 12, 3, 3, 10, 7, -1},
        {17, 23, 19, 24, 17, 15, 15, 21, 20, 23, 19, 12, 6, 2, 8, 15, 17, 23, 20, 15, 17, 24, 24, 21, 20, 23, 23, 21, 20, 23, 19, 22, 16, 22, 18, 12, 12, 3, 1, 3, 4, 12, 19, 24, 24, 21, 16, 23, 20, 12, 12, 3, 6, 12, 16, 22, 22, 21, 21, 14, 8, 15, -1},
    }, // MAP 2 local 740885
    {
        {{6, 0}, {4, 0}, {0, 0}, {1, 0}, {0, 1}, {2, 0}, {0, 2}, {4, 1}, {6, 1}, {2, 1}, {0, 3}, {3, 0}, {0, 4}, {4, 2}, {6, 2}, {4, 3}, {6, 3}, {2, 2}, {0, 5}, {2, 3}, {0, 6}, {4, 4}, {6, 4}, {4, 5}, {4, 6}, {24, 1}, {37, 8}, {46, 0}, {49, 0}, {46, 1}, {43, 0}, {32, 7}, {28, 10}, {22, 7}, {21, 5}, {38, 7}, {38, 8}, {24, 6}, {6, 11}, {4, 10}, {2, 9}, {24, 9}, {6, 13}, {2, 11}, {0, 14}, {5, 1}, {6, 14}, {4, 15}, {2, 12}, {24, 13}, {37, 15}, {40, 4}, {43, 2}, {32, 13}, {28, 14}, {11, 5}, {16, 12}, {22, 13}, {28, 15}, {33, 2}, {28, 16}, {22, 14}, {22, 15}, {24, 19}, {6, 18}, {2, 19}, {0, 19}, {2, 20}, {2, 21}, {24, 22}, {37, 20}, {46, 7}, {49, 3}, {38, 14}, {38, 15}, {24, 27}, {6, 22}, {2, 25}, {0, 22}, {4, 23}, {4, 24}, {24, 30}, {37, 24}, {33, 5}, {32, 18}, {24, 33}, {7, 2}, {11, 13}, {16, 16}, {22, 22}, {22, 23}, {24, 37}, {6, 28}, {4, 31}, {2, 27}, {24, 40}, {37, 27}, {32, 25}, {28, 27}, {22, 28}, {22, 29}, {24, 44}, -1},
        {{7, 0}, {11, 0}, {16, 1}, {22, 1}, {21, 0}, {38, 1}, {37, 1}, {38, 2}, {37, 2}, {32, 1}, {28, 5}, {32, 2}, {28, 6}, {22, 4}, {21, 3}, {40, 1}, {37, 5}, {38, 5}, {39, 2}, {32, 4}, {28, 9}, {11, 1}, {16, 7}, {22, 6}, {21, -1}, {40, 3}, {37, 7}, {32, 5}, {32, 6}, {24, 4}, {6, 8}, {2, 6}, {0, 9}, {1, 2}, {0, -1}, {2, 7}, {4, 9}, {24, 7}, {37, 10}, {32, 9}, {28, 11}, {32, 10}, {28, 12}, {11, 3}, {16, 9}, {22, 8}, {22, 9}, {24, 12}, {37, 13}, {38, 11}, {38, 12}, {24, 14}, {6, 15}, {2, 13}, {0, 15}, {2, 14}, {2, 15}, {24, 16}, {37, 16}, {38, 13}, {37, 17}, {32, 14}, {28, 17}, {12, 1}, {11, 6}, {24, 21}, {6, 19}, {2, 22}, {0, 20}, {4, 19}, {6, 20}, {4, 20}, {4, 21}, {24, 25}, {37, -1}, {38, 16}, {39, 4}, {32, 17}, {28, 21}, {22, 19}, {16, 14}, {11, 8}, {11, 9}, {24, -1}, {6, 24}, {3, 2}, {1, 6}, {24, 32}, {6, 26}, {4, 28}, {0, 26}, {1, 7}, {0, 27}, {4, -1}, {4, 30}, {24, 36}, {37, 26}, {32, 22}, {28, 25}, {22, 25}, {22, 26}, {24, 39}, {6, 30}, {2, 28}, {0, 30}, {3, 5}, {3, 6}, {24, 43}, {6, -1}, -1},
        {{28, 0}, {22, 0}, {16, 0}, {12, 0}, {16, 2}, {23, 1}, {16, 3}, {22, 2}, {28, 4}, {23, 2}, {16, 5}, {23, 3}, {21, 2}, {38, 4}, {39, 1}, {34, 0}, {28, 8}, {25, 0}, {16, 6}, {11, 2}, {16, 8}, {23, 5}, {23, 6}, {24, 2}, {6, 6}, {3, 1}, {0, 7}, {2, 4}, {2, -1}, {24, 5}, {37, 9}, {38, 9}, {39, 3}, {29, 0}, {32, -1}, {24, 8}, {6, -1}, {1, 3}, {0, 13}, {4, -1}, {4, 14}, {24, 11}, {37, 12}, {46, 2}, {49, 1}, {46, 3}, {43, 1}, {23, 7}, {16, 10}, {11, 4}, {16, 11}, {22, 11}, {22, 12}, {24, 17}, {6, 17}, {4, 17}, {0, 18}, {2, 17}, {2, -1}, {24, 20}, {37, 19}, {46, 4}, {49, 2}, {46, 5}, {46, 6}, {24, -1}, {6, -1}, {4, 22}, {0, 21}, {2, 23}, {2, 24}, {24, 26}, {37, 23}, {34, 1}, {28, 22}, {11, 10}, {16, 15}, {22, 20}, {22, 21}, {24, 31}, {6, 25}, {4, 26}, {4, 27}, {24, 34}, {37, 25}, {32, 19}, {28, -1}, {22, 24}, {28, 24}, {32, -1}, {32, 21}, {24, 38}, {6, 29}, {3, 4}, {0, 29}, {4, 32}, {4, 33}, {24, 42}, {6, 32}, {5, 4}, {0, 31}, {2, 29}, {0, -1}, -1},
        {{37, 0}, {38, 0}, {39, 0}, {32, 0}, {28, -1}, {23, 0}, {28, 2}, {33, 0}, {28, 3}, {22, 3}, {16, 4}, {13, 0}, {21, 1}, {38, 3}, {37, 3}, {40, 0}, {37, 4}, {32, 3}, {28, 7}, {23, 4}, {22, 5}, {24, 0}, {37, -1}, {40, 2}, {38, 6}, {24, 3}, {6, 5}, {4, 7}, {6, 7}, {1, 1}, {0, 8}, {4, 8}, {6, 9}, {5, 0}, {6, 10}, {2, 8}, {0, 11}, {2, 10}, {0, 12}, {4, 11}, {4, 12}, {24, 10}, {37, -1}, {32, 11}, {28, 13}, {22, 10}, {21, 6}, {38, 10}, {37, 14}, {33, 1}, {32, 12}, {24, 15}, {6, -1}, {2, 16}, {0, 16}, {1, 4}, {0, 17}, {4, 16}, {4, -1}, {24, 18}, {37, 18}, {33, 3}, {28, 18}, {22, 16}, {28, 19}, {33, -1}, {32, -1}, {24, 23}, {37, 21}, {32, 16}, {28, 20}, {11, 7}, {16, 13}, {22, 17}, {22, 18}, {24, 28}, {6, 23}, {1, 5}, {0, 23}, {2, 26}, {0, 24}, {3, 3}, {0, 25}, {4, 25}, {7, -1}, {11, 11}, {11, 12}, {24, 35}, {6, 27}, {5, 2}, {0, -1}, {5, 3}, {7, 3}, {22, 27}, {28, 26}, {32, 23}, {32, 24}, {24, 41}, {6, 31}, {4, -1}, -1},
    }, // MAP 3 local 986601
    {
        {8, 11, 8, 10, 7, 13, 8, 11, 5, 11, 8, 11, 2, 10, 7, 13, 8, 12, 3, 12, 9, 12, 2, 14, 8, 11, 2, 17, 17, 0, 8, 14, 8, 11, 8, 14, 2, 14, 5, 11, 5, 17, 8, 14, 17, 0, 5, 17, 17, 0, 2, 17, 5, -1},
        {9, 15, 9, 12, 9, 15, 11, 0, 2, 12, 9, 12, 9, 14, 12, 0, 2, 10, 7, 10, 1, 10, 2, 17, 8, 14, 5, 17, 5, 17, 17, 0, 2, 17, 17, 0, 2, 14, 6, 17, 8, 14, 8, 13, 10, 0, 0, 16, 3, -1},
        {5, 11, 5, 11, 5, 15, 9, 15, 5, 13, 7, 10, 4, 13, 5, 17, 3, 12, 6, 11, 2, 14, 14, 0, 1, 14, 11, 0, 5, 11, 11, 0, 2, 17, 14, 0, 0, 16, 5, 13, 13, 0, 5, 13, 10, -1},
        {6, 15, 8, 11, 5, 13, 7, 13, 5, 11, 2, 12, 6, 15, 2, 14, 7, 14, 11, 0, 2, 17, 14, 0, 0, 16, 8, 11, 14, 0, 0, 16, 13, 0, 2, 14, 2, 10, 7, 10, 7, 13, -1},
    }, // MAP 4 local 608422
};

int MAP_FEATURE[PRE_WORK_MAP_NUM][3] = {
    {43, 1, 5}, // MAP 1
    {25, 6, 5}, // MAP 2
    {50, 3, 5}, // MAP 3
    {18, 7, 1}, // MAP 4
};
//初赛训练赛
/*
int PRE_WORK_DATA[PRE_WORK_MAP_NUM][4][WORK_LIST_LENGTH] = {
    {
        {4, 8, 13, 10, 9, 2, 0, 2, 1, 2, 0, 2, 1, 8, 8, 14, 17, 19, 18, 25, 23, 25, 25, 14, 18, 22, 22, 14, 19, 16, 21, 19, 18, 25, 23, 25, 25, 16, 12, 5, 7, 5, 5, 16, 21, 22, 22, 16, 16, 15, 14, 15, 17, 20, 21, 22, 19, 14, 14, 15, 16, 15, 17, 20, 21, 22, 22, 16, 21, 22, 26, 22, 22, 16, 16, 15, 14, 15, 17, 19, 19, 14, 13, 5, 6, 5, 7, 10, 14, 15, 18, -1},
        {3, 8, 13, 19, 18, 25, 23, 25, 24, 25, 23, 25, 23, 25, 24, 25, 25, 14, 13, 11, 12, 8, 9, 2, 0, 2, 1, 8, 4, 10, 9, 8, 4, 8, 8, 14, 14, 15, 13, 11, 11, 16, 16, 15, 17, 19, 19, 14, 18, 25, 23, 25, 18, 20, 21, 22, 22, 16, 17, 19, 18, 25, 23, 25, 26, 22, 22, 14, 10, 14, 18, 22, 27, 19, 25, 14, 17, 20, 20, 16, 17, 25, 24, 25, 25, 16, 21, -1},
        {12, 5, 7, 5, 6, 5, 7, 5, 7, 5, 6, 5, 5, 14, 18, 20, 21, 22, 27, 22, 17, 19, 18, 25, 23, 25, 25, 14, 14, 15, 13, 10, 9, 11, 12, 10, 10, 14, 18, 22, 26, 25, 23, 25, 25, 14, 13, 5, 6, 5, 5, 16, 17, 19, 18, 22, 26, 20, 20, 16, 8, 14, 18, 20, 20, 16, 12, 8, 3, 8, 4, 10, 9, 8, 8, 16, 16, 15, 17, 19, 18, 11, 13, -1},
        {17, 20, 21, 28, 30, 28, 29, 28, 30, 28, 30, 28, 29, 28, 27, 19, 18, 22, 27, 19, 19, 14, 17, 20, 20, 16, 12, 8, 8, 16, 12, 5, 7, 5, 5, 16, 21, 22, 22, 16, 13, 10, 9, 8, 4, 8, 8, 14, 18, 22, 27, 19, 18, 22, 27, 19, 25, 14, 13, 5, 6, 5, 5, 16, 12, 5, 7, 5, 5, 16, 21, 22, 27, 19, 18, 25, 25, 14, 10, 16, 21, 10, -1},
    }, // MAP 1 local 1091264
    {
        {2, 10, 2, 9, 0, 9, 3, 13, 5, 14, 6, 13, 5, 11, 4, 13, 6, 14, 5, 14, 5, 11, 3, 11, 4, 13, 5, 11, 3, 9, 9, 15, 5, 14, 6, 14, 14, 15, 15, 16, 6, 14, 14, 15, 6, 14, 7, 13, 4, 12, 12, 15, 15, 16, 4, 12, 5, 11, 3, 11, 4, 13, 0, -1},
        {4, 12, 3, 9, 1, 10, 2, 9, 1, 10, 3, 11, 12, 15, 3, 11, 4, 12, 13, 15, 4, 10, 5, 12, 12, 15, 3, 9, 9, 15, 5, 12, 12, 15, 15, 16, 5, 12, 5, 14, 14, 15, 6, 13, 13, 15, 3, 11, 4, 13, 5, 12, 13, 15, 15, -1},
        {7, 13, 4, 12, 6, 13, 5, 12, 4, 10, 2, 9, 1, 10, 2, 10, 1, 11, 11, 15, 10, 15, 5, 10, 2, 9, 3, 11, 4, 12, 5, 11, 12, 15, 6, 13, 13, 15, 3, 13, 13, 15, 6, 14, 7, 12, 11, 15, 11, 15, -1},
        {6, 13, 5, 12, 4, 13, 6, 14, 7, 13, 13, 15, 4, 12, 5, 12, 5, 14, 6, 14, 14, 15, 3, 11, 11, 15, 15, 16, 5, 12, 3, 9, 2, 9, 2, 12, 5, 14, 7, 13, 5, 12, 4, 12, 12, 15, 15, 16, 5, 12, 5, 14, 6, 13, 5, 11, 12, 15, 15, 16, -1},
    }, // MAP 2 local 756344
    {
        {2, 5, 4, 6, 4, 9, 5, 0, 1, 6, 3, 6, 3, 6, 4, 10, 13, 9, 5, 0, 0, 11, 15, 10, 4, 6, 4, 7, 4, 6, 2, 7, 4, 7, 7, 0, 1, 5, 4, 6, 2, 5, 5, 0, 2, 5, 3, 7, 1, 6, 6, 0, 2, 7, -1},
        {1, 5, 2, 7, 2, 7, 1, 5, 4, 7, 7, 0, 1, 5, 3, 6, 2, 5, 4, 9, 13, 9, 6, 0, 1, 6, 4, 9, 3, 7, 4, 10, 13, 6, 6, 0, 2, 7, 4, 7, 1, 5, 6, 0, 0, 11, 15, 8, 8, 0, -1},
        {3, 7, 4, 9, 4, 7, 2, 7, 7, 0, 1, 6, 5, 0, 1, 6, 4, 7, 7, 0, 1, 6, 2, 7, 7, 0, 0, 11, 13, 8, 7, 0, 1, 8, 8, 0, 0, 11, 15, 8, 2, 7, 7, 0, 1, 5, 4, 6, 2, 7, 7, 0, 0, 11, 15, -1},
        {13, 9, 2, 5, 4, 7, 9, 0, 2, 5, 2, 7, 6, 0, 1, 6, 6, 0, 1, 5, 4, 6, 2, 5, 5, 0, 2, 8, 2, 7, 6, 0, 1, 5, 2, 5, 1, 5, 5, 0, 1, 6, 2, 5, 0, 11, 15, 8, 1, 8, 4, 6, -1},
    }, // MAP 3 local 711743
    {
        {2, 8, 1, 8, 1, 9, 2, 8, 1, 8, 1, 8, 8, 0, 4, 12, 3, 10, 2, 8, 1, 10, 2, 11, 3, 10, 2, 11, 3, 12, 12, 0, 3, 22, 29, 39, 29, 27, 26, 36, 26, 28, 29, 18, 17, 0, 4, 12, 4, 11, 17, 0, 3, 12, 12, 0, 3, 12, 4, 12, 18, 0, 3, 18, 18, 0, 4, 12, 4, 6, 5, 16, 5, 13, 13, 0, 3, 12, -1},
        {5, 7, 5, 13, 31, 13, 5, 13, 31, 13, 5, 13, 31, 13, 13, 0, 3, 12, 4, 12, 4, 11, 3, 17, 29, 17, 29, 27, 26, 25, 26, 27, 26, 35, 26, 27, 26, 37, 26, 20, 26, 27, 29, 18, 12, 0, 3, 18, 18, 0, 3, 17, 29, 27, 27, 0, 4, 12, 4, 11, 11, 0, 4, 11, 11, 0, 3, 12, 4, 12, 12, 0, 4, 11, 11, 0, 3, 18, 18, 0, 3, 12, 4, -1},
        {29, 30, 29, 39, 47, 42, 46, 40, 48, 42, 46, 38, 29, 39, 47, 42, 46, 40, 48, 39, 39, 0, 3, 17, 29, 17, 29, 18, 29, 42, 46, 40, 40, 0, 3, 18, 29, 40, 48, 39, 39, 0, 3, 12, 4, 11, 3, 9, 2, 11, 11, 0, 3, 17, 29, 27, 26, 19, 26, 27, 29, 11, 3, 18, 29, 17, 17, 0, 3, 17, 29, 17, 17, 0, 4, 21, 29, 17, 17, 0, 4, 12, 12, 0, -1},
        {31, 33, 31, 24, 31, 41, 48, 39, 29, 40, 48, 39, 47, 40, 48, 42, 46, 32, 29, 39, 47, 39, 29, 21, 29, 17, 3, 12, 4, 12, 4, 11, 3, 17, 29, 40, 48, 39, 39, 0, 3, 18, 29, 39, 29, 11, 11, 0, 4, 12, 12, 0, 4, 22, 29, 17, 29, 18, 29, 17, 17, 0, 3, 12, 12, 0, 3, 17, 29, 18, 12, 0, 2, 11, 3, 12, 18, 0, 3, 17, 11, 0, 3, 17, -1},
    }, // MAP 4 local 805632
};

int MAP_FEATURE[PRE_WORK_MAP_NUM][3] = {
    {31, 1, 2}, // MAP 1
    {17, 1, 2}, // MAP 2
    {18, 7, 1}, // MAP 3
    {50, 9, 1}, // MAP 4
};
*/