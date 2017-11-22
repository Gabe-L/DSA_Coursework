#include <iostream>
#include <list>
#include <string>
#include <set>
#include <vector>

using namespace std;
using std::list;

struct coord {
	int x, y;
};

struct Vertex {
	std::string name;
	bool blocked;
	int distance; //-1 if not filled in yet
	int f, g, h;
	coord coords;
	std::list<pair<Vertex*, int>> neighbours;
	Vertex* parent;

	Vertex(std::string name_, coord coord_) :
		name(name_), coords(coord_), blocked(false), distance(-1) {}
};

void connect(Vertex& a, Vertex& b, int weight) {
	a.neighbours.push_back({ &b, weight });
	b.neighbours.push_back({ &a, weight });
}

void display(std::list<Vertex *>& graph) {
	for (auto v : graph) {
		cout << v->name << ": f - " << v->f << " g - " << v->g << " h - " << v->h << endl;
	}
}

void phase1(Vertex& start, Vertex& goal, int edge) {

	for (auto v : start.neighbours) {
		if (!v.first->blocked) {
			if (v.first->distance == -1 || v.first->distance > start.distance + v.second) {
				v.first->distance = start.distance + v.second;
				/*if (v.first == &goal) {
				return;
				}*/
				phase1(*v.first, goal, v.second);
			}
		}
	}
	return;
}

void phase2(Vertex* self, std::list<Vertex *>& path) {
	path.push_back(self);
	pair<Vertex*, int> betterPath;
	betterPath.second = 200;

	for (auto v : self->neighbours) {
		if (v.first->distance == 0) {
			path.push_back(v.first);
			return;
		}
		if (!v.first->blocked) {
			if (v.first->distance < self->distance && v.second < betterPath.second) {
				betterPath = v;
			}
		}
	}
	phase2(betterPath.first, path);
	return;
}

int heuristic(Vertex* node, Vertex* goal) {
	int less, D = 200, D2 = 200;
	int dx = abs(node->coords.x - goal->coords.x);
	int dy = abs(node->coords.y - goal->coords.y);

	if (dx < dy) {
		less = dx;
	}
	else {
		less = dy;
	}

	for (auto v : node->neighbours) {
		if (v.first->coords.x == node->coords.x || v.first->coords.y == node->coords.y) { //v is in direction NSEW
			if (v.second < D) {
				D = v.second;
			}
		}
		else { //v is inbetween an NSEW direction
			if (v.second < D2) {
				D2 = v.second;
			}
		}
	}

	if (D == 200) { D = 0; }
	if (D2 == 200) { D2 = 0; }
	int h = D * (dx + dy) + (D2 - 2 * D) * less;

	return h;
}

Vertex* lowestF(set<Vertex*> open) {
	Vertex* low = *open.begin();

	for (auto v : open) {
		if (v->f < low->f) {
			low = v;
		}
	}
	return low;
}

std::list<Vertex *> aStar(Vertex* start, Vertex* end) {
	std::set<Vertex*> open, close;
	start->g = 0;
	start->f = start->g + heuristic(start, end);

	/*FOR TESTING*/
	start->h = heuristic(start, end);
	/*-----------------------------*/

	Vertex* current;
	int new_g;

	open.insert(start);

	while (!open.empty()) {
		current = lowestF(open);

		if (current == end) {
			break;
		}

		open.erase(current);
		close.insert(current);

		for (auto v : current->neighbours) {
			if (close.count(v.first) > 0) {
				continue;
			}

			new_g = current->g + v.second;

			if (open.count(v.first) < 1) {
				open.insert(v.first);
			}
			else if (new_g >= v.first->g) {
				continue;
			}
			v.first->g = new_g;
			v.first->f = v.first->g + heuristic(v.first, end);
			/*FOR TESTING*/
			v.first->h = heuristic(v.first, end);
			/*---------------------------------*/
			v.first->parent = current;
		}
	}

	//tracing back

	std::list<Vertex*> path;
	Vertex* track;
	track = end;
	while (track != start) {
		path.push_front(track);
		track = track->parent;
	}
	path.push_front(start);

	return path;
}

Vertex* findVertex(coord dest, std::list<Vertex*> graph) {
	for (auto node : graph) {
		if (node->coords.x == dest.x && node->coords.y == dest.y) {
			return node;
		}
	}
}

bool isVertex(coord dest, std::list<Vertex*> graph) {
	for (auto node : graph) {
		if (node->coords.x == dest.x && node->coords.y == dest.y) {
			return true;
		}
	}
	return false;
}

void generate2(std::list<Vertex*> *graph, Vertex* parent) {
	for (int i = 0; i < 3; i++) {
		coord tempCoord = { parent->coords.x + (rand() & 3), parent->coords.y + (rand() & 3) };
		graph->push_back(new Vertex(char(tempCoord.x + 65) + to_string(tempCoord.y), tempCoord));
		connect(*parent, *graph->back(), 2);
	}
	
	for (auto node : parent->neighbours) {
		generate2(graph, node.first);
	}
	return;
}

std::list<Vertex *> generateGraph(int range, int spread, bool blocks, int connections) {

	string tempName;
	std::list<Vertex*> graph;
	//vector<Vertex*> tempGraph;

	//populate the tempGraph
	for (int outer = 0; outer < range; outer++) {
		for (int inner = 0; inner < range; inner++) {
			if ((rand() % 11) > 5) {
				tempName = char(outer + 65) + to_string(inner + 1);
				graph.push_back(new Vertex(tempName, { outer,inner }));

				//set up connections
				Vertex* temp = graph.back();
				if (temp->neighbours.size() < connections) {
					//left
					for (int x = temp->coords.x - 1; x >= 0; x--) {
						if (isVertex({ x, temp->coords.y }, graph)) {
							Vertex* left = findVertex({ x, temp->coords.y }, graph);
							connect(*temp, *left, 2);
							break;
						}
					}
					//up
					for (int y = temp->coords.y - 1; y >= 0; y--) {
						if (isVertex({ temp->coords.x, y }, graph)) {
							Vertex* up = findVertex({ temp->coords.x, y }, graph);
							connect(*temp, *up, 2);
							break;
						}
					}
				}
			}
		}
	}

	return graph;
}

void test() {
	Vertex va("A", { 0,1 }), vb("B", { 1,0 }), vc("C", { 2,1 }), vd("D", { 1, 2 }), ve("E", { 3,0 }), vf("F", { 3,2 }), vg("G", { 2,3 }), vh("H", { 4,3 }), vi("I", { 4,1 }), vx("X", { 2,8 }), vy("Y", { 5,8 }), vz("Z", { 7,10 });
	connect(va, vb, 7);
	connect(vb, vc, 3);
	connect(vb, vd, 2);
	connect(vc, ve, 4);
	connect(vc, vf, 2);
	connect(vf, vh, 6);
	connect(vf, vi, 3);
	connect(vd, vg, 5);
	connect(vg, vf, 1);
	connect(vi, vh, 2);

	connect(vg, vx, 4);
	connect(vx, vy, 2);
	connect(vy, vz, 6);
	//vc.blocked = true;

	std::list<Vertex *> graph;
	graph.push_back(new Vertex("A5", { 0,5 }));
	
	generate2(&graph, graph.front());
	
	va.distance = 0;
	//phase1(va, vh, 0);

	std::list<Vertex *> path;
	/*phase2(&vh, path);
	path.reverse();
	cout << "Distances:" << endl;*/

	path = aStar(&va, &vh);

	display(graph);

	cout << endl << "Path taken:" << endl;

	display(path);
}

int main() {

	test();

	//wait for user input to end program
	cin.get();

	return 0;

}