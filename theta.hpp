#pragma once
#include <vector>

struct point {
    short x;
    short y;
    point(short _x, short _y) {
        x = _x;
        y = _y;
    }
    operator int() const {
        return *(int*)this;
    }
};

void theta(point p_start, point p_goal, std::vector<point>& walls, std::vector<point>& path);
void print(point p_start, point p_goal, std::vector<point>& walls, std::vector<point>& path);
