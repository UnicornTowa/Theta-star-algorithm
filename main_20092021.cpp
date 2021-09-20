#include <iostream>
#include <vector>

using namespace std;

struct node_str;
struct path_str;

struct node_str {
    int x;
    int y;
    int g_score{};
    int h_score{};
    int total_score;
    node_str* parent;
    path_str* neighbour;
    node_str(int x, int y) : x(x), y(y) {}
    friend ostream& operator<< (ostream &out, const node_str &_node_str);
};

ostream& operator<< (ostream &out, const node_str &_node_str){
    out << "X = " << _node_str.x << ", Y = " << _node_str.y << ", g_score = " <<
        _node_str.g_score <<", h_score = " << _node_str.h_score;

    return out;
}


struct vector_node : vector<node_str*> {
    inline auto bin_search(node_str* s) {
        return lower_bound(begin(), end(), s, [](const node_str* a, const node_str* b) {
            return a->total_score < b->total_score;
        });
    }
    inline bool has(node_str* s) {
        auto it = bin_search(s);
        return it != end() && *it == s;
    }
};
struct sorted_node : vector_node {
    inline void remove(node_str* s) {
        auto it = bin_search(s);
        if (it != end() && *it == s) erase(it);
    }
    inline void add(node_str* s) {
        auto it = bin_search(s);
        insert(it, s);
    }
};
struct path_str {
    node_str* x0;
    node_str* x1;
    friend ostream& operator<< (ostream &out, const path_str &_path_str);
};

ostream& operator<< (ostream &out, const path_str &_path_str) {
    out << "X:" << _path_str.x0->x << ", Y:" << _path_str.x0->y << " <=> X:" << _path_str.x1->x << ", Y:" << _path_str.x1->y;
    return out;
}


vector<node_str> node;
vector<path_str> path;
sorted_node open;
vector_node closed;
vector<node_str*> reverse_path;


inline int c(node_str* a, node_str* b) {
    int x = b->x - a->x;
    int y = b->y - a->y;
    double cc = x*x + y*y;
    return sqrt(cc);
}

inline void prepare(node_str* goal) {
    qsort(&path[0], path.size(), sizeof(path_str), [](const void* a, const void* b) -> int {
        return ((path_str*)a)->x0 - ((path_str*)b)->x0;
    });

    node_str* last = 0;
    for (path_str& p : path) {
        node_str* current = p.x0;
        if (last != current) {
            last = current;
            last->neighbour = &p;
        }
    }

    for (node_str& n : node) {
        n.h_score = c(&n, goal);
        cout << n << endl;
    }
}

inline void reconstruct_path(node_str* s) {
    do {
        reverse_path.push_back(s);
        s = s->parent;
    } while (s);
}
inline void update_vertex(node_str* s, node_str* neighbour) {
    bool line_of_sight = false;
    node_str* parent = s->parent;
    if (parent) {
        path_str* p = parent->neighbour;
        do {
            if (p->x1 == neighbour) {
                line_of_sight = true;
                break;
            }
            p++;
        } while (p->x0 == parent);
    }

    if (line_of_sight) {
        int score = parent->g_score + c(parent, neighbour);
        if (score < neighbour->g_score) {
            open.remove(neighbour);
            neighbour->g_score = score;
            neighbour->total_score = score + neighbour->h_score;
            neighbour->parent = parent;
            open.add(neighbour);
        }
    }
    else {
        int score = s->g_score + c(s, neighbour);
        if (score < neighbour->g_score) {
            open.remove(neighbour);
            neighbour->g_score = score;
            neighbour->total_score = score + neighbour->h_score;
            neighbour->parent = s;
            open.add(neighbour);
        }
    }
}
static void theta(node_str* start, node_str* goal) {
    start->g_score = 0;
    start->parent = 0;

    open.push_back(start);

    do {
        node_str* s = open.back();
        open.pop_back();
        if (s == goal) {
            reconstruct_path(s);
            return;
        }
        closed.push_back(s);

        path_str* p = s->neighbour;
        if (p) do {
                node_str* neighbour = p->x1;
                if (!closed.has(neighbour)) {
                    if (!open.has(neighbour)) {
                        neighbour->g_score = INT_MAX;
                        neighbour->parent = 0;
                    }
                    update_vertex(s, neighbour);
                }
                p++;
            } while (p->x0 == s);
    } while (!open.empty());
}

void fill_nodes(vector<node_str>* _node) // чтобы заполнять квадрат с произвольной стороной X надо заменить 400 на (X - 1) * 100
{
    for (int Ycoord = 0; Ycoord <= 400; Ycoord += 100) {
        for (int Xcoord = 0; Xcoord <=400; Xcoord += 100) {
            _node->push_back({Xcoord + 0, Ycoord + 0});
        }
    }
}
void fill_connections(vector<path_str>* _path) // чтобы заполнять квадрат с произвольной стороной X надо заменить все 5 на X, все 20 на X * (X-1), 24 на X^2 - 1, 4 на X - 1
{
    for (int i = 0; i < 24; i ++)
    {
        if ((i - 4) % 5 != 0)
        {
            _path->push_back({&node[i + 0], &node[i + 1]});
            if (i < 20){
                _path->push_back({&node[i + 0], &node[i + 6]});
            }
        }
        if (i < 20) {
            _path->push_back({&node[i + 0], &node[i + 5]});
            if (i % 5 != 0) {
                _path->push_back({&node[i + 0], &node[i + 4]});
            }
        }

    }
}

int main()
{
    fill_nodes(&node);
    //cout << node[1].g_score << endl;
    fill_connections(&path);

    //for(auto & i : path)
    //    cout << i << endl;

    prepare(&node[5]);
    //cout << "0" << node.size() << "1";
    //for(auto & i : node) {
    //    cout << node.size();
    //    cout << "----------------" << i << endl;
    //}

    theta(&node[0], &node[5]);

    return 0;
}

