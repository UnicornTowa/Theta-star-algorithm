#include <iostream>
#include <algorithm>
#include "theta.h"
using namespace std;

constexpr int DistanceStep = 16;

struct cell : point {
    uint16_t g_score;
    uint16_t total_score;
    cell* parent;
    cell(short x, short y, uint16_t _g_score, uint16_t _total_score, cell* _parent)
    : point(x, y)
    {
        g_score = _g_score;
        total_score = _total_score;
        parent = _parent;
    }
};

inline bool wall_has(vector<point>& wall, point p) {
    auto it = lower_bound(wall.begin(), wall.end(), p);
    return it != wall.end() && *it == p;
}

struct close_vec : vector<cell> {
    inline cell* find(point p) {
        for (cell& c : *this)
            if (c == p)
                return &c;
        return 0;
    }
};
struct open_vec : close_vec {
    inline auto bin_search(cell& s) {
        return lower_bound(begin(), end(), s, [](const cell& a, const cell& b) {
            return a.total_score > b.total_score;
        });
    }
    inline bool has(cell& s) {
        auto it = bin_search(s);
        return it != end() && *it == s;
    }
    inline void remove(cell& s) {
        auto it = bin_search(s);
        if (it != end() && *it == s) erase(it);
    }
    inline void add(cell& s) {
        auto it = bin_search(s);
        insert(it, s);
    }
};


inline uint32_t distance(point& a, point& b) {
    int x = b.x - a.x;
    int y = b.y - a.y;
    double cc = x * x + y * y;
    return (uint32_t)(sqrt(cc) * DistanceStep);
}

bool line_of_sight(point a, point b, vector<point>& wall) {
    int dx, dy, sx, sy;
    if (a.x > b.x) {
        dx = (int)a.x - b.x;
        sx = -1;
    }
    else {
        dx = (int)b.x - a.x;
        sx = 1;
    }
    if (a.y > b.y) {
        dy = (int)a.y - b.y;
        sy = -1;
    }
    else {
        dy = (int)b.y - a.y;
        sy = 1;
    }
    dx++;
    dy++;
    int f = dx - dy;
    if (f < 0) f++;

    while (true) {
        if (f > 0) {
            f -= dy;
            a.x += sx;
        }
        else {
            f += dx;
            a.y += sy;
        }
        if (a == b) return true;
        if (wall_has(wall, a)) return false;
    };
}



void reconstruct_path(vector<point>& path, cell* goal) {
    while (true) {
        path.push_back(*goal);
        cell* next = goal->parent;
        if (goal == next) return;
        goal = next;
    }
}
void update_vertex(cell* s, point n, point goal, open_vec& opened, close_vec& closed, vector<point>& wall) {
    if (closed.find(n))
        return;

    cell* parent = s->parent;
    uint16_t new_g;
    if (line_of_sight(*parent, n, wall)) {
        s = parent;
        new_g = distance(*s, n);
    }
    else new_g = DistanceStep;
    new_g += s->g_score;

    cell* n_ptr = opened.find(n);
    uint16_t total;
    if (n_ptr) {
        if (new_g >= n_ptr->g_score) return;
        opened.remove(*n_ptr);
        total = n_ptr->total_score - n_ptr->g_score + new_g;
    }
    else total = new_g + distance(n, goal);
    cell neighbour = { n.x,n.y, new_g, total, s };
    opened.add(neighbour);
}

void move_wall(close_vec &closed, vector<point> &wall) {
    int i = 0;
    while (i < size(wall)) {
        cell temp{wall[i].x, wall[i].y, 0, 0, nullptr};
        closed.push_back(temp);
        i ++;
    }
}

void theta(point p_start, point p_goal, vector<point>& wall, vector<point>& path) {
    sort(wall.begin(), wall.end());

    close_vec closed;
    closed.reserve(1024 * sizeof(cell));
    move_wall(closed, wall);

    uint16_t d = distance(p_start, p_goal);
    cell start = { p_start.x,p_start.y, 0, d, closed.end()._Ptr };

    open_vec opened;
    opened.push_back(start);

    do {
        cell s = opened.back();
        opened.pop_back();

        if (s == p_goal) {
            reconstruct_path(path, &s);
            return;
        }
        closed.push_back(s);

        update_vertex(&closed.back(), point(s.x-1, s.y), p_goal, opened, closed, wall);
        update_vertex(&closed.back(), point(s.x+1, s.y), p_goal, opened, closed, wall);
        update_vertex(&closed.back(), point(s.x, s.y+1), p_goal, opened, closed, wall);
        update_vertex(&closed.back(), point(s.x, s.y-1), p_goal, opened, closed, wall);
    } while (!opened.empty());
}

void print(point p_start, point p_goal, std::vector<point>& walls, std::vector<point>& path) {
    int x0 = INT_MAX, x1 = INT_MIN, y0 = INT_MAX, y1 = INT_MIN;
    for (point& p : walls) {
        if (x0 > p.x) x0 = p.x;
        if (x1 < p.x) x1 = p.x;
        if (y0 > p.y) y0 = p.y;
        if (y1 < p.y) y1 = p.y;
    }
    for (point& p : path) {
        if (x0 > p.x) x0 = p.x;
        if (x1 < p.x) x1 = p.x;
        if (y0 > p.y) y0 = p.y;
        if (y1 < p.y) y1 = p.y;
    }
    int width = x1 - x0 + 1;
    int height = y1 - y0 + 1;
    int stride = width + 1;

    vector<char> buffer(stride * height, ' ');
    char* ptr = &buffer.front() + width;
    do {
        *ptr = '\n';
        ptr += stride;
    } while (ptr != &buffer.back());
    *ptr = 0;

    auto set_buf = [&](point p, char c) {
        p.x -= x0;
        p.y -= y0;
        buffer[p.y * stride + p.x] = c;
    };

    for (point& p : walls)
        set_buf(p, 'X');
    for (point& p : path)
        set_buf(p, '*');
    set_buf(p_start, '@');
    set_buf(p_goal, '@');

    cout << &buffer[0] << '\n';
}
