
#include <iostream>
#include <vector>
using namespace std;

struct node_str;
struct path_str;
struct node_str {
	int x;
	int y;
	int g_score;
	int h_score;
	int total_score;
	node_str* parent;
	path_str* neighbour;
	node_str(int x, int y) : x(x), y(y) {}
};
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
};

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

	for (node_str& n : node)
		n.h_score = c(&n, goal);
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

int main()
{
	node.push_back({ 0,500 });
	node.push_back({ -100,600 });
	node.push_back({ 300,-800 });
	node.push_back({ 800,-900 });
	node.push_back({ 300,600 });
	node.push_back({ -700,-200 });
	node.push_back({ -600,0 });

	path.push_back({ &node[1],&node[5] });
	path.push_back({ &node[3],&node[2] });
	path.push_back({ &node[3],&node[6] });
	path.push_back({ &node[3],&node[1] });
	path.push_back({ &node[2],&node[4] });
	path.push_back({ &node[0],&node[1] });
	path.push_back({ &node[5],&node[0] });
	path.push_back({ &node[1],&node[3] });
	path.push_back({ &node[4],&node[5] });

	prepare(&node[6]);
	theta(&node[0], &node[6]);

	return 0;
}
